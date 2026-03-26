#pragma once

#include "ik_processing/srv/data.hpp"

using Datapoint = ik_processing::msg::Datapoint; 

namespace nullspace_solver{

class Input_Trajectory{

public: 

    Input_Trajectory(const std::vector<Datapoint>& data);


    double get_segment_duration(size_t segment) const;
    double get_segment_duration() const;
    int get_current_segment() const; 


private:


    std::vector<Datapoint> data_;
    int current_segment_ = 0 ;
    // double t_segment_;
    double segment_duration_;
};





} // namespace nullspace_solver