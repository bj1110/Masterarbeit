#include "ik_processing/evaluator.hpp"

namespace evaluator{

joint::joint(int arr_pos, const moveit_msgs::msg::RobotTrajectory& rt, const rclcpp::Logger& LOGGER):
LOGGER_(LOGGER) 
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
    
    jerk.resize(jtps.size());
    /* forward differantion */
    if (jtps.size() > 1) {
        double dt = step_durations[0];
        if (dt < 1e-12) {
            jerk[0] = 0.0; 
        } else {
            jerk[0] = (jtps[1].accelerations[arr_pos] - jtps[0].accelerations[arr_pos]) / dt;
        }
    /* centered differantion */
        for (size_t i = 1; i < jtps.size() - 1; ++i) {
            double dt_total = step_durations[i-1] + step_durations[i];
            if (dt_total < 1e-12) {
                jerk[i] = 0.0;
            } else {
                jerk[i] = (jtps[i+1].accelerations[arr_pos] - jtps[i-1].accelerations[arr_pos]) / dt_total;
            }
        }
    /* backward differantion */
        dt = step_durations.back();
        if (dt < 1e-12) {
            jerk.back() = 0.0;
        } else {
            jerk.back() = (jtps.back().accelerations[arr_pos] - jtps[jtps.size()-2].accelerations[arr_pos]) / dt;
        }
    } else {
        jerk[0] = 0.0; 
    }

    double jerk_integral = 0.0;
    for (size_t i = 0; i < step_durations.size(); ++i) {
        jerk_integral += jerk[i] * jerk[i] * step_durations[i];
    }

    NJS = std::sqrt(0.5* (std::pow(full_movement_duration, 5.0) / std::pow(movement_length, 2)) * jerk_integral);
}

double calculate_total_NJS(const moveit_msgs::msg::RobotTrajectory& rt, const rclcpp::Logger& LOGGER){
    std::vector<joint> joints;
    double num_joints  = rt.joint_trajectory.joint_names.size();
    for(int i=0; i<num_joints ; ++i){
        joints.emplace_back(i, rt, LOGGER); 
    }
    double NJS=0;
    for(const auto& j:joints){
        NJS += j.get_NJS(); 
        RCLCPP_INFO(LOGGER, "new value: %f", NJS);
    }
    return NJS; 
}
double calculate_av_NJS(const moveit_msgs::msg::RobotTrajectory& rt, const rclcpp::Logger& LOGGER){
    double num_joints  = rt.joint_trajectory.joint_names.size();
    double NJS_total = calculate_total_NJS(rt, LOGGER);
    RCLCPP_INFO(LOGGER, "\033[31m NJS total %f \033[0m", NJS_total);
    return NJS_total / num_joints; 
}

}