#include "ik_processing/evaluator.hpp"
#include "ik_processing/evaluator.hpp"

namespace evaluator{

joint::joint(int arr_pos, const moveit_msgs::msg::RobotTrajectory& rt, const rclcpp::Logger& logger):
LOGGER_(logger) 
{
    const auto& jt = rt.joint_trajectory; 
    const auto& jtps = jt.points; 
    name=jt.joint_names[arr_pos];
    full_movement_duration = helpers::to_double(jtps.back().time_from_start - jtps.front().time_from_start); 
    movement_length= (jtps.back().positions[arr_pos] - jtps.front().positions[arr_pos]);

    jerk.resize(jtps.size());
    step_durations.resize(jtps.size() - 1);
    for (size_t i = 0; i < step_durations.size(); ++i) {
        step_durations[i] = helpers::to_double(
            jtps[i+1].time_from_start - jtps[i].time_from_start
        );
    }
    std::vector<double> accelerations;
    accelerations.reserve(jtps.size());
    if (jtps[0].accelerations.empty()){
        // RCLCPP_INFO_STREAM(LOGGER_, "No acceleration entries for " << name <<" calculating via numeric derivation");
        std::vector<double> velocities;
        for(size_t i=0; i<jtps.size(); ++i){
            velocities.push_back(jtps[i].velocities[arr_pos]);
        }
        accelerations = numeric_derivitive(velocities, step_durations);
    } else{
        for(size_t i=0; i< jtps.size(); ++i){
            accelerations.push_back(jtps[i].accelerations[arr_pos]);
        }
    }

    jerk= numeric_derivitive(accelerations, step_durations);

    double jerk_integral = 0.0;
    for (size_t i = 0; i < step_durations.size(); ++i) {
        if(!(std::isfinite(jerk[i]))){
            RCLCPP_WARN_STREAM(LOGGER_, "infinite jerk value encountered in " << name<<". skipping"); 
            continue; 
        }
        jerk_integral += jerk[i] * jerk[i] * step_durations[i];
    }

    NJS = std::sqrt(0.5* (std::pow(full_movement_duration, 5.0) / std::pow(movement_length, 2)) * jerk_integral);
    if(movement_length < 1e-6){
        NJS=0;
    }
}

    std::vector<double> joint::numeric_derivitive(const std::vector<double>& in, const std::vector<double>& step_durations) const{
    std::vector<double> res;
    res.resize(in.size());
    if(in.empty()){
        RCLCPP_ERROR(this->LOGGER_, "Vector to derive is empty, cannot calculate jerk");
        return {};
    }
    double dt = step_durations[0];
    if (dt < 1e-12) {
        res[0] = 0.0; 
    } else {
        res[0] = (in[1] - in[0]) / dt;
    }
    for (size_t i = 1; i < in.size() - 1; ++i) {
        double dt_total = step_durations[i-1] + step_durations[i];
        if (dt_total < 1e-12) {
            res[i] = 0.0;
        } else {
            res[i] = (in[i+1] - in[i-1]) / dt_total;
        }
    }
    dt = step_durations.back();
    if (dt < 1e-12) {
        res.back() = 0.0;
    } else {
        res.back() = (in.back()- in[in.size()-2]) / dt;
    }
    return res; 
}

double calculate_total_NJS(const moveit_msgs::msg::RobotTrajectory& rt, const rclcpp::Logger& logger, LogLevel loglevel){
    std::vector<joint> joints;
    double num_joints  = rt.joint_trajectory.joint_names.size();
    for(int i=0; i<num_joints ; ++i){
        joints.emplace_back(i, rt, logger); 
    }
    double NJS=0;
    for(const auto& j:joints){
        RCLCPP_INFO_STREAM_EXPRESSION (logger, loglevel==LogLevel::verbose ,"" << j.get_name() << ": " <<j.get_NJS()); 
        NJS += j.get_NJS(); 
    }
    return NJS; 
}
double calculate_av_NJS(const moveit_msgs::msg::RobotTrajectory& rt, const rclcpp::Logger& logger, LogLevel loglevel){
    double num_joints  = rt.joint_trajectory.joint_names.size();
    double NJS_total = calculate_total_NJS(rt, logger, loglevel);
    double NJS_av = NJS_total / num_joints; 
    if(loglevel== LogLevel::error){
        return NJS_av; 
    }
    RCLCPP_INFO(logger, "\033[33m NJS total %.2f \033[0m", NJS_total);
    return NJS_av; 
}

endeffector::endeffector(const std::vector<geometry_msgs::msg::Pose>& waypoints, const std::vector<double>& timesteps, const rclcpp::Logger& logger):
LOGGER_(logger)
{
    movementduration_ = timesteps.back() - timesteps.front(); 
    positions_.reserve(waypoints.size());
    velocities_.reserve(waypoints.size());
    accelerations_.reserve(waypoints.size());
    jerks_.reserve(waypoints.size());
    timestepsizes_.reserve(timesteps.size()-1); 
    for(const auto& wp:waypoints){
        Eigen::Vector3d vec;
        vec= {wp.position.x, wp.position.y, wp.position.z}; 
        positions_.push_back(vec);
    }
    for(size_t i=0; i<timesteps.size() -1; ++i){
        timestepsizes_.push_back(timesteps[i+1] - timesteps[i]);
    }
    velocities_ = numeric_derivitive(positions_);
    accelerations_ = numeric_derivitive(velocities_);
    jerks_ = numeric_derivitive(accelerations_);

    NJS_=calculate_NJS(); 
    RCLCPP_INFO(LOGGER_, "\033[33m NJS total of input path: %.2f \033[0m", NJS_);
}

std::vector<Eigen::Vector3d> endeffector::numeric_derivitive(const std::vector<Eigen::Vector3d> in){
    std::vector<Eigen::Vector3d> res;

    if(in.empty()){
        RCLCPP_ERROR(LOGGER_, "Vector to derive is empty, cannot calculate jerk of endeffector");
        return {};
    }
    
    res.reserve(in.size());
    double dt = timestepsizes_.front();
    if( dt < 1e-12){
        res.emplace_back(0,0,0); 
    } else{
        res.push_back( (in[1] - in[0]) /  dt);
    }

    for(size_t i=1; i<in.size()-1; ++i){
        dt = timestepsizes_[i-1] + timestepsizes_[i];
        if(dt < 1e-12){
            res.emplace_back(0,0,0);
            continue;
        }
        res.push_back( (in[i+1] - in[i-1]) / dt);
    }

    dt = timestepsizes_.back();
    if(dt< 1e-12){
        res.emplace_back(0,0,0);
        return res;
    }
    res.push_back((in.back() - in[in.size() -2 ]) / dt); 
    return res; 
}
double endeffector::calculate_NJS(){
    double movement_length = (positions_.back() - positions_.front()).norm(); 
    double jerk_integral = 0.0;
    for(size_t i=0; i<jerks_.size(); ++i){
        double jerksize = jerks_[i].norm();
        jerk_integral += jerksize * jerksize * timestepsizes_[i]; 
    }

    double NJS= std::sqrt(0.5* (std::pow(movementduration_, 5) / std::pow(movement_length, 2)) * jerk_integral );
    return NJS; 
}
double endeffector::get_NJS() const{
    return NJS_; 
}
} // namespace evaluator