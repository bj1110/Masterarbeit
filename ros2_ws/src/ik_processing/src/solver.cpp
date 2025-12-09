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

    geometry_msgs::msg::Pose target_pose = move_group.getCurrentPose().pose;
    // move_group.setStartStateToCurrentState(); 
    // auto const target_pose = []{
    // geometry_msgs::msg::Pose msg;
    //     msg.orientation.w = 1.0;
    //     msg.position.x = 0.39;
    //     msg.position.y = 0.22;
    //     msg.position.z = 0.93;
    //     return msg;
    // }();
    move_group.setPoseTarget(target_pose);
    
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    
    bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);


    RCLCPP_INFO(LOGGER, "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");


    rclcpp::shutdown();
    return 0;
}
