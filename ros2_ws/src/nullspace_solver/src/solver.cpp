#include "nullspace_solver/solver.hpp"
#include <iostream>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>

namespace nullspace_solver{

    bool solver::initFromURDF(const std::string& urdf_path, const std::string& ee_frame)
    {
    pinocchio::urdf::buildModel(urdf_path, model_);
    data_ = pinocchio::Data(model_);

    ee_frame_id_ = model_.getFrameId(ee_frame);

    if (ee_frame_id_ == (pinocchio::FrameIndex)(-1))
    {
        std::cerr << "End-effector frame not found\n";
        return false;
    }

    return true;
    }

} // namespace nullspace_solver 