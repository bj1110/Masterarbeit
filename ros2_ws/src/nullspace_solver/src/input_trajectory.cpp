#include "nullspace_solver/input_trajectory.hpp"
#include <limits>

namespace nullspace_solver{

Input_Trajectory::Input_Trajectory(const std::vector<geometry_msgs::msg::Pose>& datapoints, const std::vector<double>& timestaps): 
datapoints_(datapoints), timestamps_(timestaps)
{
    if(datapoints_.size() < 2)
        throw std::runtime_error("Not enough trajectory points");
    if(datapoints_.size() != timestamps_.size()){
        throw std::runtime_error("Different number of Positions and timestamps");
    }
    num_points_ = datapoints_.size(); 
}

Eigen::Vector3d Input_Trajectory::get_position_goal(size_t idx) const{
    const auto pos = datapoints_[idx];
    return to_eigen_vector(pos); 
}


Eigen::Vector3d Input_Trajectory::to_eigen_vector(geometry_msgs::msg::Pose pose) const{
    Eigen::Vector3d pos {pose.position.x, pose.position.y, pose.position.z}; 
    return pos; 
}

Eigen::Matrix3d Input_Trajectory::get_orientation_goal(size_t idx) const{
    geometry_msgs::msg::Quaternion q = datapoints_[idx].orientation;
    Eigen::Quaternion quat {q.w, q.x, q.y, q.z};
    return quat.normalized().toRotationMatrix(); 
}

size_t Input_Trajectory::get_num_points() const{
    return num_points_; 
}






} // namespace nullspace_solver
