#pragma once

#include <Eigen/Dense>
#include "geometry_msgs/msg/pose.hpp"


namespace nullspace_solver{

class Input_Trajectory{

public: 

    Input_Trajectory(const std::vector<geometry_msgs::msg::Pose>& datapoints, const std::vector<double>& timestaps);
    Input_Trajectory() = default; 

    double calculate_segment_duration(size_t segment) const;
    double get_segment_duration() const;
    int get_current_segment_index() const; 
    Eigen::Vector3d get_current_segment() const;

    Eigen::Vector3d get_current_goalpos(double time);

    bool advance_segment();
    bool all_points_have_been_served()const; 

private:
    Eigen::Vector3d to_eigen_vector(geometry_msgs::msg::Pose dp)const;

    Eigen::Vector3d interpolate(const Eigen::Vector3d& p1,const Eigen::Vector3d& v1, const Eigen::Vector3d& p2, const Eigen::Vector3d& v2, double perc)const;

    double percentage_between_points(const size_t index1, const size_t index2, const double total_time); 

    Eigen::Vector3d approx_v(size_t seg) const;


private:

    bool allPointsServed_ = false; 
    std::vector<geometry_msgs::msg::Pose> datapoints_;
    std::vector<double> timestamps_; 
    size_t current_segment_ = 0 ;
    // double t_segment_;
    double segment_duration_;
};





} // namespace nullspace_solver