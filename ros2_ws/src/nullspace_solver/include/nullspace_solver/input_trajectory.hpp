#pragma once

#include "ik_processing/srv/data.hpp"
#include <Eigen/Dense>

using Datapoint = ik_processing::msg::Datapoint; 

namespace nullspace_solver{

class Input_Trajectory{

public: 

    Input_Trajectory(const std::vector<Datapoint>& data, bool is_agent1);
    Input_Trajectory() = default; 

    double calculate_segment_duration(size_t segment) const;
    double get_segment_duration() const;
    int get_current_segment_index() const; 
    Eigen::Vector3d get_current_segment() const;

    Eigen::Vector3d get_current_goalpos(double time, double dt);

    bool advance_segment();
    bool all_points_have_been_served()const; 

private:
    Eigen::Vector3d to_eigen_vector(Datapoint dp)const;

    Eigen::Vector3d interpolate(const Eigen::Vector3d& p1,const Eigen::Vector3d& v1, const Eigen::Vector3d& p2, const Eigen::Vector3d& v2, double perc)const;

    double Input_Trajectory::percentage_between_points(const Datapoint& dp1, const Datapoint& dp2, const double total_time); 

    Eigen::Vector3d approx_v(size_t seg) const;


private:

    bool allPointsServed_ = false; 
    std::vector<Datapoint> data_;
    int current_segment_ = 0 ;
    // double t_segment_;
    double segment_duration_;
    bool is_agent1_ = true; 
};





} // namespace nullspace_solver