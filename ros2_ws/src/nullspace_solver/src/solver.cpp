#include "nullspace_solver/solver.hpp"
#include <iostream>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include "pinocchio/algorithm/joint-configuration.hpp"
#include <cassert>

namespace nullspace_solver{

bool Solver::solve(const Eigen::VectorXd& start_configuration, const Eigen::Isometry3d& goal, moveit_msgs::msg::RobotTrajectory& trajectory){
    trajectory.joint_trajectory.joint_names = model_.names; 
    double time = 0.0;
    
    Eigen::Vector3d curr_goal = input_traj_.get_current_goalpos(time+ dt_); 
    pinocchio::SE3 oMdes(goal.rotation(), curr_goal);
    Eigen::VectorXd q = start_configuration; 

    pinocchio::Data::Matrix6x J(6, model_.nv);
    J.setZero();

    bool success= false; 
    typedef Eigen::Matrix<double, 6, 1> Vector6d;
    Vector6d err;
    Eigen::VectorXd v(DoF_); // v= q' 
    Eigen::MatrixXd J_pinv(DoF_, 6);
    Eigen::MatrixXd v_primary(DoF_);
    Eigen::MatrixXd N(DoF_, DoF_);
    Eigen::VectorXd v_secondary(DoF_);

    int iteration=0; 

    while(!input_traj_.all_points_have_been_served()){ 
        curr_goal = input_traj_.get_current_goalpos(time+ dt_); 
        oMdes.translation() = curr_goal; 
        pinocchio::forwardKinematics(model_, data_, q);
        const pinocchio::SE3 dMi = oMdes.actInv(data_.oMi[ee_frame_id_]);
        err=pinocchio::log6(dMi).toVector();
        //possibily weigh position/orientation: err.tail<3>() *= 0.1;  // reduce orientation importance
        if(err.norm() < eps_){
            success=true;
            break;
        }
        if(++iteration >= max_steps_){
            success=false;
            break;
        }
        if(time >= max_time_){
            success=false;
            break; 
        }

        compute_weighted_J_pinv(J,q,J_pinv);
        
        v_primary.noalias() = J_pinv * err ; 

        N.noalias() = Eigen::MatrixXd::Identity(DoF_, DoF_) - J_pinv* J;
        nullspaceObjective(q, v_secondary);

        v.noalias() = v_primary + N*v_secondary; 
        q=pinocchio::integrate(model_, q, v*dt_);

        trajectory_msgs::msg::JointTrajectoryPoint point = create_JTP(q,v,time);
        trajectory.joint_trajectory.points.push_back(point);
        time += dt_; 

        // if(!(i%10)){
        //     std::cout << i << ": error = " << err.transpose() << std::endl;
        // }
    }
            
    // std::cout << "\nresult: " << q.transpose() << std::endl;
    // std::cout << "\nfinal error: " << err.transpose() << std::endl;
    return success; 
}

Solver::Solver(const std::string& urdf_path, const std::string& ee_frame, std::vector<Datapoint> input_data, bool is_agent1):
    input_traj_(input_data, is_agent1)
{
    initFromURDF(urdf_path, ee_frame);
    DoF_= model_.nv;
    W_inv_ = Eigen::MatrixXd::Identity(DoF_, DoF_); 
    max_time_= input_data.back().time + 1.0; 
}

Solver::Solver(const std::string& urdf_path, const std::string& ee_frame):
    input_traj_()
{
    initFromURDF(urdf_path, ee_frame);
    DoF_= model_.nv;
    W_inv_ = Eigen::MatrixXd::Identity(DoF_, DoF_); 
}


bool Solver::initFromURDF(const std::string& urdf_path, const std::string& ee_frame){
    pinocchio::urdf::buildModel(urdf_path, model_);
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
    pinocchio::computeFrameJacobian(model_, data_, q, ee_frame_id_, pinocchio::LOCAL_WORLD_ALIGNED, J); //Frame instead of JointJacobian...
    pinocchio::Data::Matrix6 JWJt;
    JWJt.noalias() = J * W_inv_* J.transpose();  // J* W^{-1}*J^t
    JWJt.diagonal().array() += damp_; // JJ^t + Lambda*I
    J_pinv.noalias() = W_inv_ * J.transpose() * JWJt.ldlt().solve(Eigen::MatrixXd::Identity(6,6));//W^{-1} * J^t ( JJ^t + Lambda*I)^{-1}        
}

trajectory_msgs::msg::JointTrajectoryPoint Solver::create_JTP(const Eigen::VectorXd& q, const Eigen::VectorXd& v, const double time) const{
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.resize(DoF_);
    point.velocities.resize(DoF_);  
    for(int j=0; j<DoF_; ++j){
        point.positions[j] = q[j];
        point.velocities[j] = v[j]; 
    }
    point.time_from_start = rclcpp::Duration::from_seconds(time);
    return point; 
}



} // namespace nullspace_solver 