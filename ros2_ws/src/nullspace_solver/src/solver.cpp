#include "nullspace_solver/solver.hpp"
#include <iostream>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include "pinocchio/algorithm/joint-configuration.hpp"
#include <cassert>

namespace nullspace_solver{

bool solver::solve(const Eigen::VectorXd& start_configuration, const Eigen::Isometry3d& goal, moveit_msgs::msg::RobotTrajectory& trajectory, const double dt ){
    const pinocchio::SE3 oMdes(goal.rotation(), goal.translation());
    Eigen::VectorXd q = start_configuration; 

    pinocchio::Data::Matrix6x J(6, model_.nv);
    J.setZero();

    bool success= false; 
    typedef Eigen::Matrix<double, 6, 1> Vector6d;
    Vector6d err;
    Eigen::VectorXd v(model_.nv); // v= q' 

    trajectory.joint_trajectory.joint_names = model_.names; 
    double time = 0.0;

    for(int i=0;;++i){
        pinocchio::forwardKinematics(model_, data_, q);
        const pinocchio::SE3 dMi = oMdes.actInv(data_.oMi[ee_frame_id_]);
        err=pinocchio::log6(dMi).toVector();
        //possibily weigh position/orientation: err.tail<3>() *= 0.1;  // reduce orientation importance
        if(err.norm() < eps_){
            success=true;
            break;
        }
        if(i >= max_steps_){
            success=false;
            break;
        }

        Eigen::MatrixXd J_pinv;
        compute_weighted_J_pinv(J,q,J_pinv);
        
        Eigen::MatrixXd v_primary;
        v_primary.noalias() = J_pinv * err ; 

        Eigen::MatrixXd N;
        N.noalias() = Eigen::MatrixXd::Identity(DoF_, DoF_) - J_pinv* J;
        Eigen::VectorXd v_secondary(DoF_);
        nullspaceObjective(q, v_secondary);

        v.noalias() = v_primary + N*v_secondary; 
        q=pinocchio::integrate(model_, q, v*dt);

        trajectory_msgs::msg::JointTrajectoryPoint point = create_JTP(q,v,time);
        trajectory.joint_trajectory.points.push_back(point);
        time += dt; 

        // if(!(i%10)){
        //     std::cout << i << ": error = " << err.transpose() << std::endl;
        // }
    }
            
    // std::cout << "\nresult: " << q.transpose() << std::endl;
    // std::cout << "\nfinal error: " << err.transpose() << std::endl;
    return success; 
}

solver::solver(const std::string& urdf_path, const std::string& ee_frame, std::vector<Datapoint> input_data, bool is_agent1):
    in_traj_(input_data, is_agent1)
{
    initFromURDF(urdf_path, ee_frame);
    DoF_= model_.nv;
    W_inv_ = Eigen::MatrixXd::Identity(DoF_, DoF_); 
}

solver::solver(const std::string& urdf_path, const std::string& ee_frame):
    in_traj_()
{
    initFromURDF(urdf_path, ee_frame);
    DoF_= model_.nv;
    W_inv_ = Eigen::MatrixXd::Identity(DoF_, DoF_); 
}


bool solver::initFromURDF(const std::string& urdf_path, const std::string& ee_frame){
    pinocchio::urdf::buildModel(urdf_path, model_);
    data_ = pinocchio::Data(model_);
    ee_frame_id_ = model_.getFrameId(ee_frame);
    if (ee_frame_id_ == (pinocchio::FrameIndex)(-1)){
        return false;
    }
    return true;
}

void solver::setJointLimits(const Eigen::VectorXd& q_min, const Eigen::VectorXd& q_max){
    q_min_ = q_min;
    q_max_ = q_max;
}


bool solver::adjust_weight(const size_t pos, const double weight){
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

bool solver::adjust_weight(const Eigen::VectorXd& weights){
    assert(weights.size() == W_inv_.diagonalSize());
    if(weights.size() != W_inv_.diagonalSize()){
        return false;
    }
    W_inv_.diagonal() = weights; 
    return true;
}

void solver::compute_weighted_J_pinv(pinocchio::Data::Matrix6x& J, const Eigen::VectorXd& q, Eigen::MatrixXd& J_pinv){
    pinocchio::computeJointJacobian(model_, data_, q, ee_frame_id_, J);
    pinocchio::Data::Matrix6 JWJt;
    JWJt.noalias() = J * W_inv_* J.transpose();  // J* W^{-1}*J^t
    JWJt.diagonal().array() += damp_; // JJ^t + Lambda*I
    J_pinv.noalias() = W_inv_ * J.transpose() * JWJt.ldlt().solve(Eigen::MatrixXd::Identity(6,6));//W^{-1} * J^t ( JJ^t + Lambda*I)^{-1}        
}

trajectory_msgs::msg::JointTrajectoryPoint solver::create_JTP(const Eigen::VectorXd& q, const Eigen::VectorXd& v, const double time) const{
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.resize(DoF_);
    for(int j=0; j<DoF_; ++j){
        point.positions[j] = q[j];
        point.velocities[j] = v[j]; 
    }
    point.time_from_start = rclcpp::Duration::from_seconds(time);
    return point; 
}



} // namespace nullspace_solver 