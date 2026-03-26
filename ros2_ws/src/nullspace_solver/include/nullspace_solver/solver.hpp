#pragma once

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include "nullspace_solver/input_trajectory.hpp"



namespace nullspace_solver{

class solver{
public: 
    bool solve(const Eigen::VectorXd& start_configuration, const Eigen::Isometry3d& goal, moveit_msgs::msg::RobotTrajectory& trajectory, const double dt );

    void setJointLimits(const Eigen::VectorXd& q_min, const Eigen::VectorXd& q_max);

    bool initFromURDF(const std::string& urdf_path, const std::string& ee_frame);

    solver()=default;
    ~solver() = default;
    solver(const solver& other) = delete;
    solver(solver&& other)=default;
    solver& operator=(const solver& other) = delete;
    solver& operator=(solver&& other)=default; 
    solver(const std::string& urdf_path, const std::string& ee_frame,  std::vector<Datapoint> input_data, bool is_agent1);
    solver(const std::string& urdf_path, const std::string& ee_frame);


    bool adjust_weight(const size_t pos, const double weight);
    bool adjust_weight(const Eigen::VectorXd& weights);


private:
    
    void nullspaceObjective(const Eigen::VectorXd& q, Eigen::VectorXd& objective );

    void compute_weighted_J_pinv(pinocchio::Data::Matrix6x& J, const Eigen::VectorXd& q, Eigen::MatrixXd& J_pinv);

    trajectory_msgs::msg::JointTrajectoryPoint create_JTP(const Eigen::VectorXd& q, const Eigen::VectorXd& v, const double time) const; 



private:
    pinocchio::Model model_;
    pinocchio::Data data_;
    pinocchio::FrameIndex ee_frame_id_; //could also be int


    int DoF_; 
    Eigen::MatrixXd K_;
    Eigen::MatrixXd W_inv_; 
    double lambda_ = 1e-6; 
    int max_steps_= 1000;
    double eps_ = 1e-4;
    double damp_ = 1e-6; 


    Eigen::VectorXd q_min_;
    Eigen::VectorXd q_max_;

    Input_Trajectory in_traj_; 
};
} // namespace nullspace_solver 