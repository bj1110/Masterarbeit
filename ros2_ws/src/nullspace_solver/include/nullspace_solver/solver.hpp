#pragma once

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include "nullspace_solver/input_trajectory.hpp"



namespace nullspace_solver{

class Solver{
public: 
    bool solve(const Eigen::VectorXd& start_configuration, const Eigen::Isometry3d& goal, moveit_msgs::msg::RobotTrajectory& trajectory);

    void setJointLimits(const Eigen::VectorXd& q_min, const Eigen::VectorXd& q_max);

    bool initFromURDF(const std::string& urdf_path, const std::string& ee_frame);

    Solver()=default;
    ~Solver() = default;
    Solver(const Solver& other) = delete;
    Solver(Solver&& other)=default;
    Solver& operator=(const Solver& other) = delete;
    Solver& operator=(Solver&& other)=default; 
    Solver(const std::string& urdf_path, const std::string& ee_frame, const std::vector<Datapoint>& input_data, const bool is_agent1);
    Solver(const std::string& urdf_path, const std::string& ee_frame);


    bool adjust_weight(const size_t pos, const double weight);
    bool adjust_weight(const Eigen::VectorXd& weights);


private:
    
    void nullspaceObjective(const Eigen::VectorXd& q, Eigen::VectorXd& objective );

    void compute_weighted_J_pinv(pinocchio::Data::Matrix6x& J, const Eigen::VectorXd& q, Eigen::MatrixXd& J_pinv);

    trajectory_msgs::msg::JointTrajectoryPoint create_JTP(const Eigen::VectorXd& q, const Eigen::VectorXd& v, const double time) const; 

    void check_joint_boundary(Eigen::VectorXd& v);


private:
    pinocchio::Model model_;
    pinocchio::Data data_;
    pinocchio::FrameIndex ee_frame_id_; //could also be int


    int DoF_; 
    Eigen::MatrixXd K_;
    Eigen::MatrixXd W_inv_; 
    int max_steps_= 1000;
    double eps_ = 1e-4;
    double damp_ = 1e-6; 
    double max_time_ = 10;
    double dt_ = 0.001; 
    double margin_ = 1e-4;  


    Eigen::VectorXd q_min_;
    Eigen::VectorXd q_max_;

    Input_Trajectory input_traj_; 
};
} // namespace nullspace_solver 