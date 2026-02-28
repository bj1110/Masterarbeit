#pragma once 
#include <memory>
#include <experimental/propagate_const>
#include <rclcpp/rclcpp.hpp> 
#include <Eigen/Dense>
		

namespace rigid_body_algorithm{

class crb{
    private:
    class impl;
    std::experimental::propagate_const<std::unique_ptr<impl>> pimpl;

    public: 
    crb();
    ~crb();
    crb(crb&&);
    crb(const crb&) = delete;
    crb& operator=(const crb&) = delete;
    crb& operator=(crb&&); 
    crb(const std::vector<double>& joint_states, const std::string& urdf_string, const rclcpp::Logger& logger);

    bool crba();
    Eigen::MatrixXd get_inertia_Matrix() const; 

    bool update_model_state(const std::vector<double> joint_states);




};



} //namespace rigid_body_algorithm