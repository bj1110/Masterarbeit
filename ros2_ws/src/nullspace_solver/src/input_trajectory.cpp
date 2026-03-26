#include "nullspace_solver/input_trajectory.hpp"
#include <limits>

namespace nullspace_solver{

Input_Trajectory::Input_Trajectory(const std::vector<Datapoint>& data): data_(data)
{
    segment_duration_ = get_segment_duration(0); 
}

double Input_Trajectory::get_segment_duration() const{
    return segment_duration_;
}

int Input_Trajectory::get_current_segment() const{
    return current_segment_; 
}

double Input_Trajectory::get_segment_duration(size_t segment) const{
    if(segment >= data_.size()-2){
        return std::numeric_limits<double>::quiet_NaN();
    }
    double durr = data_[segment+1].time - data_[segment].time; 
}   









} // namespace nullspace_solver
