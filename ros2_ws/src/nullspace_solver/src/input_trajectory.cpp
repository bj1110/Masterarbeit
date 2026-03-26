#include "nullspace_solver/input_trajectory.hpp"
#include <limits>

namespace nullspace_solver{

Input_Trajectory::Input_Trajectory(const std::vector<Datapoint>& data, bool is_agent1): data_(data), is_agent1_(is_agent1)
{
    segment_duration_ = calculate_segment_duration(0); 
}

double Input_Trajectory::get_segment_duration() const{
    return segment_duration_;
}

int Input_Trajectory::get_current_segment_index() const{
    return current_segment_; 
}


double Input_Trajectory::calculate_segment_duration(size_t segment) const{
    if(segment >= data_.size()-1){
        return std::numeric_limits<double>::quiet_NaN();
    }
    double durr = data_[segment+1].time - data_[segment].time; 
    return durr; 
}   

bool Input_Trajectory::advance_segment(){
    if(all_points_have_been_served()){
        return false;
    }
    if(current_segment_ >= data_.size() -1 ){
        allPointsServed_= true; 
        return false;  
    }
    current_segment_ ++; 
    if(current_segment_ >= data_.size() - 1){
        allPointsServed_ = true;
    }
    segment_duration_= calculate_segment_duration(current_segment_);
    return true; 
}

bool Input_Trajectory::all_points_have_been_served() const{
    return allPointsServed_; 
}

Eigen::Vector3d Input_Trajectory::get_current_segment() const{
    return to_eigen_vector(data_[current_segment_]); 
}

Eigen::Vector3d Input_Trajectory::get_current_goalpos(double time, double /*dt*/){
    if(all_points_have_been_served()){
        return to_eigen_vector(data_.back());
    }
    if(current_segment_>=data_.size()-1){
        return to_eigen_vector(data_.back());
    }
    while(current_segment_ < data_.size()-1 && time > data_[current_segment_+1].time) {
        advance_segment();
    }
    if(current_segment_>=data_.size()-1){
        return to_eigen_vector(data_.back());
    }
    Datapoint dp1 = data_[current_segment_];
    Datapoint dp2 = data_[current_segment_+1];
    double total_time = time /*+dt*/ ;
    
    double perc = percentage_between_points(dp1, dp2, total_time);
 
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
        Eigen::Vector3d p1 = to_eigen_vector(data_[seg + 1]);
        Eigen::Vector3d p0 = to_eigen_vector(data_[seg]);
        return (p1 - p0) / (data_[seg + 1].time - data_[seg].time);
    }
    if (seg == data_.size() - 1) {
        // Backward difference
        Eigen::Vector3d p1 = to_eigen_vector(data_[seg]);
        Eigen::Vector3d p0 = to_eigen_vector(data_[seg - 1]);
        return (p1 - p0) / (data_[seg].time - data_[seg - 1].time);
    }
    // Central difference
    Eigen::Vector3d p_l = to_eigen_vector(data_[seg - 1]);
    Eigen::Vector3d p_r = to_eigen_vector(data_[seg + 1]);
    double t_l = data_[seg - 1].time;
    double t_r = data_[seg + 1].time;
    return (p_r - p_l) / (t_r - t_l);
}

double Input_Trajectory::percentage_between_points(const Datapoint& dp1, const Datapoint& dp2, const double total_time){
    const double t1 = dp1.time;
    const double t2 = dp2.time;
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




Eigen::Vector3d Input_Trajectory::to_eigen_vector(Datapoint dp) const{
    Eigen::Vector3d pos;
    if(is_agent1_){
        pos={dp.x1,
            dp.y1,
            dp.z1
        };
    }else{
        pos={dp.x2,
            dp.y2,
            dp.z2
        };
    }
    return pos; 
}









} // namespace nullspace_solver
