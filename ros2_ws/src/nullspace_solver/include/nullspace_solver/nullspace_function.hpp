#pragma once

#include <functional>
#include <Eigen/Dense>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include "pinocchio/algorithm/joint-configuration.hpp"


namespace nullspace_solver{

namespace tasks{

using Nullspace_task = std::function<Eigen::VectorXd(const Eigen::VectorXd& q)>;

/**
 * Creates a Nullspace task that couples movement from the second joint to that of the first.
 * This is used if the second joint should have a multiple of the first joint's deflection.
 * The second joint will be pulled towards (couplingfactor * deflection[joint1]).
 * This does not couple both joints together, only the second joint will be influenced.
 * 
 * 
 * 
 * @param j1                Index of the joint that should be mimiked within the Vector q
 * @param j2                Index of the joint that should mimik the other joint within the Vector q                
 * @param couplingfactor    The factor that scales the coupling
 * 
 * @return                  Nullspace task that couples the movement of the second joint to that of the first joint.
 */

Nullspace_task couple_joints(const size_t j1, const size_t j2, double couplingfactor){
    return [j1, j2, couplingfactor](const Eigen::VectorXd& q){
        Eigen::VectorXd gradient = Eigen::VectorXd::Zero(q.size());
        assert((void ("Index out of bounds"), j1 < q.size() || j2 < q.size()));
        if(j1 >= q.size() || j2 >= q.size()){
            throw std::invalid_argument("Index out of bounds for couple_joints");
            return gradient;
        }
        gradient[j2] = q[j2] - couplingfactor * q[j1];
        return gradient; 
    };
}


/**
 * Creates a Nullspace task that keeps a joint near a certain jointvalue.
 * This can be used if a certain jointvalue is preferable. 
 * Make sure to use radians for revolute joints.
 * 
 * @param idx       The index of the joint that shall be nudged
 * @param value     The value that the joint shall stay near. In radians for rotating joints
 * 
 * @return          Nullspace task that keeps the joint close to the specified jointvalue
 */
Nullspace_task nudge_joint_towards_value(const size_t idx, double value){
    return [idx, value](const Eigen::VectorXd& q){
        Eigen::VectorXd gradient = Eigen::VectorXd::Zero(q.size());
        assert((void ("Index out of bounds"), idx < q.size()));
        if(idx >= q.size()){
            throw std::invalid_argument("Index out of bounds for nudge_joint_towards_value");
            return gradient;
        }
        gradient[idx] = q[idx] - value;
        return gradient;
    };
}


/**
 * Creates a Nullspace task that keeps the joint close to it's nullposition.
 * This can be used if a joint shall move as little as possible. 
 * 
 * @param idx   Index of the joint that shall be kept close to zero
 * 
 * @throws      std::invalid_argument if idx is out of bounds
 * 
 * @return      Nullspace task that keeps the joint close it's zeroposition.
 */
Nullspace_task nudge_joint_towards_nullposition(const size_t idx){
    return nudge_joint_towards_value(idx, 0.0); 
}




/**
 * Creates Nullspace task that avoids the joint limits.
 * This can be used to avoid the joint limits. However this is not a hard constraint!
 * A quadratic gradient is introduced. If the Joint gets close to it's limit (within eps) a significant errorterm is added. 
 * 
 * @param q_min     A vector of the minima of all joints
 * @param q_max     A vector of the maxima of all joints
 * @param q_mid     A vector of the midpoints of all joints
 * @param eps       If distance to jointlimit is smaller than eps the barrierterm is triggered
 * 
 * @throws          std::invalid_argument if sizes of jointlimits do not match each other or the current configuration
 * 
 * @return          Nullspace task that tries to keep the joints within limit. 
 */
Nullspace_task jointlimit_avoidance(const Eigen::VectorXd& q_min, const Eigen::VectorXd& q_max, const Eigen::VectorXd& q_mid, double eps = 0.05 ){
    if(q_min.size()!=q_max.size() || q_min.size()!= q_mid.size() || q_max.size() != q_mid.size()){
        throw std::invalid_argument("Different sizes for q_min, q_max and q_mid");
    }
    return [q_min, q_max, q_mid, eps](const Eigen::VectorXd& q){
        if(q_mid.size()!= q.size()){
            throw std::invalid_argument("Different sizes in Jointlimits vector and q.");
        }
        Eigen::VectorXd gradient = Eigen::VectorXd::Zero(q.size());
        for(size_t i=0; i<q.size(); ++i){
            //gradient[i]=4.0*pow(q[i]-q_mid[i], 3);
            double d_min = q[i] - q_min[i];
            double d_max = q_max[i] - q[i];

            if(d_min < eps){
                gradient[i] += - (eps - d_min) / (eps * eps);
            }
            if(d_max < eps){
                gradient[i] += + (eps - d_max) / (eps * eps);
            }
        }
        return gradient;
    };
}





} // namespace tasks
} //namespace nullspace_solver