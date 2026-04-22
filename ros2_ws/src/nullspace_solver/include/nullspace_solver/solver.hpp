#pragma once

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include "nullspace_solver/input_trajectory.hpp"


#include "nullspace_solver/nullspace_function.hpp"



namespace nullspace_solver{

using Nullspace_task = std::function<Eigen::VectorXd(const Eigen::VectorXd& q)>;

struct Nullspace_Objective{
    Nullspace_task task;
    double weight;
};

struct SolverConfig{
    int max_steps= 100000;
    double eps = 0.001;
    double damp = 1e-6; 
    double max_time = 10;
    double dt = 0.001; 
    double margin = 1e-4;  
    double storing_intervall = 0.01;
    double joint_limit_avoidance_gain= 1.0;
    double overall_nullspace_task_importance = 0.1;
};

class Solver{
public: 
    bool solve(const Eigen::VectorXd& start_configuration, moveit_msgs::msg::RobotTrajectory& trajectory, const bool DEBUG=false);
    void setJointLimits(const Eigen::VectorXd& q_min, const Eigen::VectorXd& q_max);

    bool initFromURDF(const std::string& urdf_string, const std::string& ee_frame);

    Solver()=default;
    ~Solver() = default;
    Solver(const Solver& other) = delete;
    Solver(Solver&& other)=default;
    Solver& operator=(const Solver& other) = delete;
    Solver& operator=(Solver&& other)=default; 
    Solver(const std::string& urdf_string, const std::string& ee_frame, const std::vector<geometry_msgs::msg::Pose>& input_data, const std::vector<double> timestamps, const rclcpp::Logger& logger);
    Solver(const std::string& urdf_string, const std::string& ee_frame, const rclcpp::Logger& logger);


    bool setJointWeight(const size_t pos, const double weight);
    bool setJointWeight(const Eigen::VectorXd& weights);

    void addNullspaceObjective(Nullspace_task task, double weight);

    void useJointLimitAvoidance();
    void centerJoint(const size_t idx, const double weight);
    void alignJointWithAxis(const std::string& j1, const Eigen::Vector3d& link_axis_in_frame, const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, double weight);
    void alignNormal(const std::string& S_name, const std::string& E_name, const std::string& W_name, double weight);

    double calculate_angle_between_plane_normal_and_z_axis(const std::string& S_name, const std::string& E_name, const std::string& W_name, const bool degrees=false);

private:
    
    void nullspaceObjective(const Eigen::VectorXd& q, Eigen::VectorXd& v);

    void compute_weighted_J_pinv(pinocchio::Data::Matrix6x& J, const Eigen::VectorXd& q, Eigen::MatrixXd& J_pinv);

    trajectory_msgs::msg::JointTrajectoryPoint create_JTP(const Eigen::VectorXd& q, const Eigen::VectorXd& v, const double time) const; 

    void check_joint_boundary(Eigen::VectorXd& v, const Eigen::VectorXd& q, bool& log);
    void avoid_joint_boundary(Eigen::VectorXd& v, const Eigen::VectorXd& q);

    bool load_config(const std::string& path); 


private:
    pinocchio::Model model_;
    pinocchio::Data data_;
    pinocchio::FrameIndex ee_frame_id_; 

    const rclcpp::Logger& LOGGER;


    size_t DoF_; 
    Eigen::MatrixXd K_;
    Eigen::MatrixXd W_inv_; 


    SolverConfig sc_; 

    std::vector<Nullspace_Objective> nullspace_objectives; 

    Eigen::VectorXd q_min_;
    Eigen::VectorXd q_max_;
    Eigen::VectorXd q_mid_;
    Eigen::VectorXd joint_ranges_;

    Input_Trajectory input_traj_; 
};
} // namespace nullspace_solver 