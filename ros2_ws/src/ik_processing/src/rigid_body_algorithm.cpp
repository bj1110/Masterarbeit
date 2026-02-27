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
    impl (const std::vector<double>& joint_states, std::string urdf_path, const rclcpp::Logger& logger):
    logger_(logger)
    {
        model_ = std::make_unique<RigidBodyDynamics::Model>();
        if (!RigidBodyDynamics::Addons::URDFReadFromFile (urdf_path.c_str(), model_.get(), false)) {
            RCLCPP_ERROR(logger_, "Error loading model %s in for crba" ,urdf_path.c_str());
            abort();
        }

        model_state_ (joint_states.data()); 
        joint_space_intertia_Matrix_ = RigidBodyDynamics::Math::MatrixNd(); 
        joint_space_intertia_Matrix_.setZero(); 
    };

};

crb::crb()= default; 
crb::~crb()=default;
crb& crb::operator=(crb&&) = default;
crb::crb(crb&&)=default;







}
