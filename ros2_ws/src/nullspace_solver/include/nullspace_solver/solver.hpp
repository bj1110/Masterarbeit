#pragma once

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>


namespace nullspace_solver{

class solver{
public: 
    bool solve(const Eigen::Isometry3d& start, const Eigen::Isometry3d& goal, moveit_msgs::msg::RobotTrajectory& trajectory );

    void setJointLimits(const Eigen::VectorXd& q_min, const Eigen::VectorXd& q_max);

    bool initFromURDF(const std::string& urdf_path, const std::string& ee_frame);



private:
    void computeError(const Eigen::Isometry3d& current, const Eigen::Isometry3d& target, Eigen::VectorXd& error);

    void computeJacobian(const Eigen::VectorXd& q, Eigen::MatrixXd& J);

    void dampedPseudoInverse(const Eigen::MatrixXd& J, Eigen::MatrixXd& J_pinv );

    void nullspaceObjective(const Eigen::VectorXd& q, Eigen::VectorXd& objective );

    void forwardKinematics(const Eigen::VectorXd& q, Eigen::Isometry3d& FK);



private:
    pinocchio::Model model_;
    pinocchio::Data data_;
    pinocchio::FrameIndex ee_frame_id_;


    u_int DoF_; 
    Eigen::MatrixXd K_;
    Eigen::MatrixXd W_;
    Eigen::MatrixXd W_inv_;
    double lambda_; 

    Eigen::VectorXd q_min_;
    Eigen::VectorXd q_max_;

    Eigen::MatrixXd J_;
    Eigen::MatrixXd J_pinv_;
    Eigen::MatrixXd N_; 
    Eigen::VectorXd error_;
    Eigen::VectorXd q_dot_; // q_dot = J_pinv * p_dot + N_ * q_dot_null;

};
} // namespace nullspace_solver 