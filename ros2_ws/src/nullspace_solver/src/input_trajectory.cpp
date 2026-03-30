#include "nullspace_solver/input_trajectory.hpp"
#include <limits>

namespace nullspace_solver{

Input_Trajectory::Input_Trajectory(const std::vector<geometry_msgs::msg::Pose>& datapoints, const std::vector<double>& timestaps): 
datapoints_(datapoints), timestamps_(timestaps)
{
    segment_duration_ = calculate_segment_duration(0); 
    if(datapoints_.size() < 2)
        throw std::runtime_error("Not enough trajectory points");
    if(datapoints_.size() != timestamps_.size()){
        throw std::runtime_error("Different number of Positions and timestamps");
    }
}

double Input_Trajectory::get_segment_duration() const{
    return segment_duration_;
}

int Input_Trajectory::get_current_segment_index() const{
    return current_segment_; 
}


double Input_Trajectory::calculate_segment_duration(size_t segment) const{
    if(segment >= timestamps_.size()-1){
        return std::numeric_limits<double>::quiet_NaN();
    }
    double durr = timestamps_[segment+1] - timestamps_[segment]; 
    return durr; 
}   

bool Input_Trajectory::advance_segment(){
    if(all_points_have_been_served()){
        return false;
    }
    if(current_segment_ >= datapoints_.size() -1 ){
        allPointsServed_= true; 
        return false;  
    }
    current_segment_ ++; 
    if(current_segment_ >= datapoints_.size() - 1){
        allPointsServed_ = true;
    }
    segment_duration_= calculate_segment_duration(current_segment_);
    return true; 
}

bool Input_Trajectory::all_points_have_been_served() const{
    return allPointsServed_; 
}

Eigen::Vector3d Input_Trajectory::get_current_segment() const{
    return to_eigen_vector(datapoints_[current_segment_]); 
}

Eigen::Vector3d Input_Trajectory::get_current_goalpos(double time){
    if(all_points_have_been_served()){
        return to_eigen_vector(datapoints_.back());
    }
    if(current_segment_>=datapoints_.size()-1){
        return to_eigen_vector(datapoints_.back());
    }
    while(current_segment_ < datapoints_.size()-1 && time > timestamps_[current_segment_+1]) {
        advance_segment();
    }
    if(current_segment_>=datapoints_.size()-1){
        return to_eigen_vector(datapoints_.back());
    }
    geometry_msgs::msg::Pose dp1 = datapoints_[current_segment_];
    geometry_msgs::msg::Pose dp2 = datapoints_[current_segment_+1];
    double total_time = time;
    
    double perc = percentage_between_points(current_segment_, current_segment_+1, total_time);
 
    Eigen::Vector3d p1 = to_eigen_vector(dp1);
    Eigen::Vector3d p2 = to_eigen_vector(dp2);
    Eigen::Vector3d v1 = approx_v(current_segment_);
    Eigen::Vector3d v2 = approx_v(current_segment_+1); 

    Eigen::Vector3d goalpos = interpolate(p1, v1, p2, v2, perc);
 
    return goalpos; 
}

Eigen::Vector3d Input_Trajectory::approx_v(size_t seg) const {
    if (seg == 0) {
        // Forward difference
        Eigen::Vector3d p1 = to_eigen_vector(datapoints_[seg + 1]);
        Eigen::Vector3d p0 = to_eigen_vector(datapoints_[seg]);
        return (p1 - p0) / (timestamps_[seg + 1] - timestamps_[seg]);
    }
    if (seg == datapoints_.size() - 1) {
        // Backward difference
        Eigen::Vector3d p1 = to_eigen_vector(datapoints_[seg]);
        Eigen::Vector3d p0 = to_eigen_vector(datapoints_[seg - 1]);
        return (p1 - p0) / (timestamps_[seg] - timestamps_[seg - 1]);
    }
    // Central difference
    Eigen::Vector3d p_l = to_eigen_vector(datapoints_[seg - 1]);
    Eigen::Vector3d p_r = to_eigen_vector(datapoints_[seg + 1]);
    double t_l = timestamps_[seg - 1];
    double t_r = timestamps_[seg + 1];
    return (p_r - p_l) / (t_r - t_l);
}

double Input_Trajectory::percentage_between_points(const size_t index1, const size_t index2, const double total_time){
    const double t1 = timestamps_[index1];
    const double t2 = timestamps_[index2];
    if (t2 <= t1) {
        return (total_time <= t1) ? 0.0 : 1.0;
    }
    if (total_time <= t1) return 0.0;
    if (total_time >= t2) return 1.0;

    return (total_time - t1) / (t2 - t1);
}   


Eigen::Vector3d Input_Trajectory::interpolate(
    const Eigen::Vector3d& p1,
    const Eigen::Vector3d& v1,
    const Eigen::Vector3d& p2,
    const Eigen::Vector3d& v2,
    double perc
) const {
    double h00 = 2 * perc * perc * perc - 3 * perc * perc + 1;
    double h10 = perc * perc * perc - 2 * perc * perc + perc;
    double h01 = -2 * perc * perc * perc + 3 * perc * perc;
    double h11 = perc * perc * perc - perc * perc;

    return h00 * p1 + h10 * v1 + h01 * p2 + h11 * v2;
}




Eigen::Vector3d Input_Trajectory::to_eigen_vector(geometry_msgs::msg::Pose pose) const{
    Eigen::Vector3d pos {pose.position.x, pose.position.y, pose.position.z}; 
    return pos; 
}

Eigen::Matrix3d Input_Trajectory::get_orientation_goal() const{
    geometry_msgs::msg::Quaternion q = datapoints_[current_segment_].orientation;
    Eigen::Quaternion quat {q.w, q.x, q.y, q.z};
    return quat.normalized().toRotationMatrix(); 
}







} // namespace nullspace_solver
