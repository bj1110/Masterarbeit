#include "ik_processing/rigid_body_algorithm.hpp"

#include <rbdl/rbdl.h>
#include <rbdl/rbdl_utils.h>
#include <rbdl/addons/urdfreader/urdfreader.h>


namespace rigid_body_algorithm{

class crb::impl{
    private:
    std::unique_ptr<RigidBodyDynamics::Model> model_;
    RigidBodyDynamics::Math::VectorNd model_state_; 
    RigidBodyDynamics::Math::MatrixNd joint_space_intertia_Matrix_;
    const rclcpp::Logger& logger_; 
    
    public: 
    impl (const std::vector<double>& joint_states, const std::string& urdf_string, const rclcpp::Logger& logger):
    logger_(logger)
    {
        model_ = std::make_unique<RigidBodyDynamics::Model>();
        if (!RigidBodyDynamics::Addons::URDFReadFromString (urdf_string.c_str(), model_.get(), false)) {
            RCLCPP_ERROR(logger_, "Error loading model from string in for crba");
            abort();
        }

        model_state_ = Eigen::Map<const Eigen::VectorXd>(
            joint_states.data(),
            joint_states.size()
        );
        joint_space_intertia_Matrix_ = RigidBodyDynamics::Math::MatrixNd(); 
        joint_space_intertia_Matrix_.setZero(); 
    };

    bool crba(){
        if(model_ == nullptr || model_state_.isZero()){
            return false;
        }
        joint_space_intertia_Matrix_.setZero(); 
        RigidBodyDynamics::CompositeRigidBodyAlgorithm(*model_, model_state_, joint_space_intertia_Matrix_);
        return true; 
    }

    Eigen::MatrixXd get_inertia_Matrix() const{
        Eigen::MatrixXd erg = joint_space_intertia_Matrix_;
        return erg; 
    }

    bool update_model_state(const std::vector<double> joint_states){
        model_state_ = Eigen::Map<const Eigen::VectorXd>(
            joint_states.data(),
            joint_states.size()
        );
        return true; 
    }

};

crb::crb()= default; 
crb::~crb()=default;
crb& crb::operator=(crb&&) = default;
crb::crb(crb&&)=default;

crb::crb(const std::vector<double>& joint_states, const std::string& urdf_string, const rclcpp::Logger& logger){
    pimpl = std::make_unique<impl>(joint_states, urdf_string, logger);
}

bool crb::crba() {
    bool success = pimpl->crba(); 
    return success; 
}

Eigen::MatrixXd crb::get_inertia_Matrix() const{
    return pimpl ->get_inertia_Matrix(); 
}

bool crb::update_model_state(const std::vector<double> joint_states){
    return pimpl -> update_model_state(joint_states); 
}





} //namespace rigid_body_algorithm
