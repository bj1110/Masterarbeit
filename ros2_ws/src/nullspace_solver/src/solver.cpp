#include "nullspace_solver/solver.hpp"
#include <iostream>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include "pinocchio/algorithm/joint-configuration.hpp"

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
        pinocchio::computeJointJacobian(model_, data_, q, ee_frame_id_, J);
        pinocchio::Data::Matrix6 JJt;
        JJt.noalias() = J * W_inv_* J.transpose();  // J* W^{-1}*J^t
        JJt.diagonal().array() += damp_; // JJ^t + Lambda*I
        v.noalias() = W_inv_* J.transpose() * JJt.ldlt().solve(err); //W^{-1} * J^t ( JJ^t + Lambda*I)^{-1} err  
        q=pinocchio::integrate(model_, q, v*dt);
        if(!(i%10)){
            std::cout << i << ": error = " << err.transpose() << std::endl;
        }
    }
    if(success){
        std::cout<< "Convergence achieved" <<std::endl;
    } else{
        std::cout << "\nWarning: the iterative algorithm has not reached convergence to the desired precision" << std::endl;
    }
            
    std::cout << "\nresult: " << q.transpose() << std::endl;
    std::cout << "\nfinal error: " << err.transpose() << std::endl;
    return success; 
}

solver::solver(const std::string& urdf_path, const std::string& ee_frame){
    initFromURDF(urdf_path, ee_frame);
    DoF_= model_.nv;
    W_inv_ = Eigen::MatrixXd::Identity(DoF_, DoF_); 
}


bool solver::initFromURDF(const std::string& urdf_path, const std::string& ee_frame){
    pinocchio::urdf::buildModel(urdf_path, model_);
    data_ = pinocchio::Data(model_);
    ee_frame_id_ = model_.getFrameId(ee_frame);
    if (ee_frame_id_ == (pinocchio::FrameIndex)(-1)){
        std::cerr << "End-effector frame not found\n";
        return false;
    }
    return true;
}

void solver::setJointLimits(const Eigen::VectorXd& q_min, const Eigen::VectorXd& q_max){
    q_min_ = q_min;
    q_max_ = q_max;
}


bool solver::adjust_weight(const size_t pos, const double weight){
    if(weight<=0 || weight >1.0){
        std::cout << "New weight not within legal range"<< std::endl;
        return false;
    }
    if (pos >= DoF_){
        std::cout << "Index not within legal range"<< std::endl;
        return false;
    }
    W_inv_(pos, pos)= weight; 
}
    

} // namespace nullspace_solver 