#include "nullspace_solver/solver.hpp"
#include <iostream>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include "pinocchio/algorithm/joint-configuration.hpp"
#include <cassert>
#include <cmath> 
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace nullspace_solver{

bool Solver::solve(const Eigen::VectorXd& start_configuration, moveit_msgs::msg::RobotTrajectory& trajectory){
    RCLCPP_INFO(LOGGER, "\033[33m Solver received Task\033[0m"); 
    for (pinocchio::JointIndex i = 1; i < model_.njoints; ++i){
        const auto& joint = model_.joints[i];
        if (joint.nq() > 0){
            trajectory.joint_trajectory.joint_names.push_back(model_.names[i]);
        }
    } 
    double time = 0.0;

    size_t wp_idx=0; 
    size_t num_wp= input_traj_.get_num_points(); 
    
    const Eigen::Matrix3d goal_orientation = input_traj_.get_orientation_goal(wp_idx);
    Eigen::Vector3d curr_goal = input_traj_.get_position_goal(wp_idx); 
    pinocchio::SE3 oMdes(goal_orientation, curr_goal);
    Eigen::VectorXd q = start_configuration; 

    RCLCPP_INFO_STREAM(LOGGER, "startgoal: " << oMdes);
    Eigen::Vector3d ee_pos = data_.oMf[ee_frame_id_].translation();
    RCLCPP_INFO_STREAM(LOGGER, " ee position" << ee_pos.transpose());  

    pinocchio::Data::Matrix6x J(6, model_.nv);
    J.setZero();

    bool success= false; 
    typedef Eigen::Matrix<double, 6, 1> Vector6d;
    Vector6d err;
    Eigen::VectorXd v(DoF_); // v= q' 
    Eigen::MatrixXd J_pinv(DoF_, 6);
    Eigen::VectorXd v_primary(DoF_);
    Eigen::MatrixXd N(DoF_, DoF_);
    Eigen::VectorXd v_secondary(DoF_);

    int iteration=0; 
    while(wp_idx < num_wp){ 
        curr_goal = input_traj_.get_position_goal(wp_idx); 
        oMdes.translation() = curr_goal; 
        pinocchio::forwardKinematics(model_, data_, q);
        if(!(iteration%1000)){
            RCLCPP_INFO_STREAM(LOGGER, "turn: "<<iteration << " q: "<<q);
        }
        pinocchio::updateFramePlacements(model_, data_);
        const pinocchio::SE3 dMi = data_.oMf[ee_frame_id_].actInv(oMdes);
        if(!(iteration%1000)){
            RCLCPP_INFO_STREAM(LOGGER, "turn: "<<iteration << " dMi: " <<dMi); 
            Eigen::Vector3d ee_pos = data_.oMf[ee_frame_id_].translation();
            RCLCPP_INFO_STREAM(LOGGER, "turn: "<<iteration << " ee position" << ee_pos.transpose()); 
        }
        err=pinocchio::log6(dMi).toVector();
        err.head<3>() *= 1.0;
        err.tail<3>() *= 0.1;  // reduce orientation importance
        if(err.head<3>().norm() < sc_.eps_ + sc_.margin_){
            if(++wp_idx == num_wp){
                success=true; 
                break; 
            }
            iteration=0;
            continue;
        }
        if(iteration >= sc_.max_steps_){
            success=false;
            RCLCPP_INFO(LOGGER, "\033[31m NUMBER OF TRIES REACHED. Failed at point %ld of %ld \033[0m", wp_idx, num_wp);        
            RCLCPP_INFO_STREAM(LOGGER, "Error at failure: "<< err.transpose());  
            RCLCPP_INFO_STREAM(LOGGER, "Positional error norm at failure: "<<err.head<3>().norm());    
            break;
        }

        compute_weighted_J_pinv(J,q,J_pinv);

        if(!(iteration%1000)){
            RCLCPP_INFO_STREAM(LOGGER, "Turn "<< iteration << " J:\n" << J);
        }
        
        v_primary.noalias() = J_pinv * err ; 

        if(!(iteration%1000)){
            auto test = J_pinv * err ;
            RCLCPP_INFO_STREAM(LOGGER, "J_pinv * err: " << test.transpose());
        }

        N.noalias() = Eigen::MatrixXd::Identity(DoF_, DoF_) - J_pinv* J;
        nullspaceObjective(q, v_secondary);

        v.noalias() = v_primary + N*v_secondary; 
        //check_joint_boundary(v, q); 
        /*
            maybe lowpass filter v, something like:
            v = 0.8 * v_prev + 0.2 * v;
        */
        if(!(iteration%1000)){
            RCLCPP_INFO_STREAM(LOGGER, "Turn "<< iteration << " v: " << v.transpose());
        }
        q=pinocchio::integrate(model_, q, v*sc_.dt_);

        trajectory_msgs::msg::JointTrajectoryPoint point = create_JTP(q,v,time);
        trajectory.joint_trajectory.points.push_back(point);
        if(!(iteration%1000)){
            RCLCPP_INFO_STREAM(LOGGER, "Turn "<< iteration <<" Error : " << err.transpose()); 
        }
        time += sc_.dt_; 
        iteration++; 
    }

    return success; 
}

void Solver::nullspaceObjective(const Eigen::VectorXd& q, Eigen::VectorXd& v){
    v.setZero(); 
}

Solver::Solver(const std::string& urdf_string, const std::string& ee_frame, const std::vector<geometry_msgs::msg::Pose>& input_data, const std::vector<double> timestamps, const rclcpp::Logger& logger):
    LOGGER(logger), input_traj_(input_data, timestamps)
{
    initFromURDF(urdf_string, ee_frame);
    DoF_= model_.nv;
    W_inv_ = Eigen::MatrixXd::Identity(DoF_, DoF_); 

    std::string config_path = ament_index_cpp::get_package_share_directory("nullspace_solver")
        + "/config/solver_config.yaml";
    load_config(config_path); 
    sc_.max_time_= timestamps.back() + 1.0; 
    RCLCPP_INFO(LOGGER, "\033[33m Solver setup complete\033[0m"); 
}

Solver::Solver(const std::string& urdf_string, const std::string& ee_frame, const rclcpp::Logger& logger):
    LOGGER(logger), input_traj_()
{
    initFromURDF(urdf_string, ee_frame);
    DoF_= model_.nv;
    W_inv_ = Eigen::MatrixXd::Identity(DoF_, DoF_); 
    std::string config_path = ament_index_cpp::get_package_share_directory("nullspace_solver")
        + "/config/solver_config.yaml";
    load_config(config_path); 
}


bool Solver::initFromURDF(const std::string& urdf_string, const std::string& ee_frame){
    pinocchio::urdf::buildModelFromXML(urdf_string, model_);
    data_ = pinocchio::Data(model_);
    ee_frame_id_ = model_.getFrameId(ee_frame);
    if (ee_frame_id_ == (pinocchio::FrameIndex)(-1)){
        return false;
    }
    return true;
}

void Solver::setJointLimits(const Eigen::VectorXd& q_min, const Eigen::VectorXd& q_max){
    q_min_ = q_min;
    q_max_ = q_max;
}


bool Solver::adjust_weight(const size_t pos, const double weight){
    assert(pos < DoF_);
    assert(weight>=0 && weight<=1.0); 
    if(weight<0 || weight >1.0){
        return false;
    }
    if (pos >= DoF_){
        return false;
    }
    W_inv_(pos, pos)= weight; 
    return true; 
}

bool Solver::adjust_weight(const Eigen::VectorXd& weights){
    assert(weights.size() == W_inv_.diagonalSize());
    if(weights.size() != W_inv_.diagonalSize()){
        return false;
    }
    W_inv_.diagonal() = weights; 
    return true;
}

void Solver::compute_weighted_J_pinv(pinocchio::Data::Matrix6x& J, const Eigen::VectorXd& q, Eigen::MatrixXd& J_pinv){
    pinocchio::computeJointJacobians(model_, data_, q);
    pinocchio::computeFrameJacobian(model_, data_, q, ee_frame_id_, pinocchio::LOCAL, J); //Frame instead of JointJacobian...
    pinocchio::Data::Matrix6 JWJt;
    JWJt.noalias() = J * W_inv_* J.transpose();  // J* W^{-1}*J^t
    JWJt.diagonal().array() += sc_.damp_; // JJ^t + Lambda*I
    J_pinv.noalias() = W_inv_ * J.transpose() * JWJt.ldlt().solve(Eigen::MatrixXd::Identity(6,6));//W^{-1} * J^t ( JJ^t + Lambda*I)^{-1}        
}

trajectory_msgs::msg::JointTrajectoryPoint Solver::create_JTP(const Eigen::VectorXd& q, const Eigen::VectorXd& v, const double time) const{
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.resize(DoF_);
    point.velocities.resize(DoF_);  
    for(size_t j=0; j<DoF_; ++j){
        point.positions[j] = q[j];
        point.velocities[j] = v[j]; 
    }
    point.time_from_start = rclcpp::Duration::from_seconds(time);
    return point; 
}

void Solver::check_joint_boundary(Eigen::VectorXd& v, const Eigen::VectorXd& q){
    if (q_min_.size() != DoF_ || q_max_.size() != DoF_) {
        RCLCPP_WARN(LOGGER, "Joint limits size (%ld, %ld) != DoF_ (%ld). Using default limits.",
                    q_min_.size(), q_max_.size(), DoF_);
        q_min_ = Eigen::VectorXd::Constant(DoF_, -M_PI);
        q_max_ = Eigen::VectorXd::Constant(DoF_, M_PI);
    }

    for(size_t i =0; i< DoF_; ++i){
        double v_min = (q_min_[i]+sc_.margin_-q[i])/sc_.dt_;
        double v_max = (q_max_[i]-sc_.margin_-q[i])/sc_.dt_;
        if (v[i] < v_min){
            v[i] = v_min;
            RCLCPP_INFO(LOGGER, "Adjusting v");
        } else if (v[i] > v_max){
            v[i] = v_max;
            RCLCPP_INFO(LOGGER, "Adjusting v");
        }
    }

}

bool Solver::load_config(const std::string& path){
    YAML::Node config = YAML::LoadFile(path);

    auto s =config["solver"];
    if(s){
        sc_.max_steps_ = s["max_steps"] ? s["max_steps"].as<int>() : sc_.max_steps_;
        sc_.eps_ = s["eps"] ? s["eps"].as<double>() : sc_.eps_;
        sc_.damp_ = s["damp"] ? s["damp"].as<double>() : sc_.damp_; 
        sc_.max_time_ = s["max_time"] ? s["max_time"].as<double>() : sc_.max_time_;
        sc_.dt_ = s["dt"] ? s["dt"].as<double>() : sc_.dt_ ; 
        sc_.margin_ = s["margin"] ? s["margin"].as<double>() : sc_.margin_;  
        return true;
    }
    RCLCPP_ERROR(LOGGER, "Config file for solver not found. Shutting down"); 
    return false;

}



} // namespace nullspace_solver 