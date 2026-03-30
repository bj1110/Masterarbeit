#pragma once

#include <Eigen/Dense>
#include "geometry_msgs/msg/pose.hpp"


namespace nullspace_solver{

class Input_Trajectory{

public: 

    Input_Trajectory(const std::vector<geometry_msgs::msg::Pose>& datapoints, const std::vector<double>& timestaps);
    Input_Trajectory() = default; 

    Eigen::Vector3d get_position_goal(size_t idx)const;

    Eigen::Matrix3d get_orientation_goal(size_t idx) const;

    size_t get_num_points()const; 

private:
    Eigen::Vector3d to_eigen_vector(geometry_msgs::msg::Pose dp)const;

private:

    std::vector<geometry_msgs::msg::Pose> datapoints_;
    std::vector<double> timestamps_; 
    size_t num_points_ = 0; 
};





} // namespace nullspace_solver