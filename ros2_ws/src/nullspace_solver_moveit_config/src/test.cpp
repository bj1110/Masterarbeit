#include "nullspace_solver/solver.hpp" 
#include <rclcpp/rclcpp.hpp>
#include <moveit/moveit_cpp/moveit_cpp.hpp>
#include <moveit/moveit_cpp/planning_component.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <Eigen/Dense>
#include <vector>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <cmath>


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto move_group_node = rclcpp::Node::make_shared("Test_Node", node_options);
    const auto LOGGER = move_group_node->get_logger(); 

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(move_group_node);
    std::thread([&executor]() { executor.spin(); }).detach();


    static const std::string PLANNING_GROUP = "arm";
    moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
    

    RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());

    std::string urdf_string = move_group_node->get_parameter("urdf").as_string(); 

    if (urdf_string.empty())
    {
        RCLCPP_ERROR(LOGGER, "robot_description parameter is empty!");
        rclcpp::shutdown(); 
        return 1;
    }


    robot_model_loader::RobotModelLoader robot_model_loader(move_group_node);
    const moveit::core::RobotModelPtr& robot_model = robot_model_loader.getModel();

    // Raw pointers are frequently used to refer to the planning group for improved performance.
    const moveit::core::JointModelGroup* joint_model_group =
        move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);


    const double link_len = 0.1; 

    geometry_msgs::msg::Pose ee_pose = move_group.getCurrentPose().pose; 
    
    RCLCPP_INFO(LOGGER, "eepose: %f %f %f", ee_pose.position.x, ee_pose.position.y, ee_pose.position.z); 

    auto x_lambda = [&link_len](double theta1, double theta2){
        return link_len* std::sin(theta1) + link_len*std::sin(theta1 + theta2);
    };
    auto z_lambda = [&link_len](double theta1, double theta2){
        return link_len* std::cos(theta1) + link_len*std::cos(theta1 + theta2);
    };


    // Example EE pose data
    std::vector<geometry_msgs::msg::Pose> input_data;
    input_data.push_back(ee_pose);
    geometry_msgs::msg::Pose p;
    for(int i=2; i <= 46; i+=2){
        p.position.x = x_lambda(i, i); p.position.y = 0.0; p.position.z = z_lambda(i, i);
        p.orientation.w = 1.0; p.orientation.x = 0.0; p.orientation.y = 0.0; p.orientation.z = 0.0;
        input_data.push_back(p);
    }
    std::vector<double> timestamps (input_data.size(), 0.0);

    std::string ee_frame = "ee";

    // Create solver instance
    nullspace_solver::Solver solver(urdf_string, ee_frame, input_data, timestamps, LOGGER);

    // Set joint limits (example for 3-DOF)
    const std::vector<std::string>& variable_names = robot_model->getVariableNames();
    size_t dof = variable_names.size();

    Eigen::VectorXd q_min(dof);
    Eigen::VectorXd q_max(dof);

    for (size_t i = 0; i < dof; ++i)
    {
        const moveit::core::VariableBounds& bounds =
            robot_model->getVariableBounds(variable_names[i]);

        q_min[i] = bounds.min_position_;
        q_max[i] = bounds.max_position_;
    }
    solver.setJointLimits(q_min, q_max);

    // Start configuration
    std::vector<double> joint_values;
    for(const std::string& name: variable_names){
        const double value = 0.0;
        joint_values.push_back(value);
    }
    Eigen::VectorXd start_configuration = Eigen::Map<Eigen::VectorXd>(joint_values.data(), dof);

    // Solve
    moveit_msgs::msg::RobotTrajectory trajectory;
    bool success = solver.solve(start_configuration, trajectory);

    if (success)
    {
        RCLCPP_INFO(LOGGER, "Trajectory successfully computed!");
        RCLCPP_INFO(LOGGER, "Trajectory has %zu points.", trajectory.joint_trajectory.points.size());
    }
    else
    {
        RCLCPP_ERROR(LOGGER, "Solver failed to compute trajectory.");
    }

    rclcpp::shutdown();
    return 0;
}