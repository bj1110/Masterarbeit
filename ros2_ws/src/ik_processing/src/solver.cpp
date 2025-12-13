#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <memory>

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    const auto move_group_node = std::make_shared<rclcpp::Node>("solver", node_options);

    const auto LOGGER = rclcpp::get_logger("solver");

    static const std::string PLANNING_GROUP = "arm"; 

    moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);

    // geometry_msgs::msg::Pose target_pose = move_group.getCurrentPose().pose;
    move_group.setStartStateToCurrentState(); 
    moveit::core::RobotState start_state(*move_group.getCurrentState());

    start_state.printStatePositions();
    start_state.printStateInfo();

    auto const target_pose = []{
    geometry_msgs::msg::Pose msg;
        msg.orientation.w = 1.0;
        msg.position.x = -0.0197;
        msg.position.y = -0.309;
        msg.position.z = 0.38;
        return msg;
    }();

    move_group.setMaxVelocityScalingFactor(0.5);
    move_group.setMaxAccelerationScalingFactor(0.5);

    move_group.setPoseTarget(target_pose);
    move_group.setPlanningTime(30.0);
    // std::map<std::string, double> joint_goal;
    // joint_goal["model1_right_shoulder_rotation_joint"] = 0.7;
    // joint_goal["model1_right_shoulder_abduction_joint"] = 1.0;
    // joint_goal["model1_right_shoulder_flexion_joint"] = 0.0;
    // joint_goal["model1_right_elbow_flexion_joint"] = 1.6113;
    // joint_goal["model1_right_elbow_rotation_joint"] = 0.0;
    // joint_goal["model1_right_wrist_flexion_joint"] = 0.0;
    // move_group.setJointValueTarget(joint_goal);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    
    bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);


    RCLCPP_INFO(LOGGER, "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");


    rclcpp::shutdown();
    return 0;
}
