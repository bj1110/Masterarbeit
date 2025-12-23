#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <chrono>
#include <thread>

#include <filesystem>
#include <stdexcept>

#include <nlohmann/json.hpp>
#include "ik_processing/msg/datapoint.hpp"

#include <mutex>

// for convenience
using json = nlohmann::json;
using Datapoint = ik_processing::msg::Datapoint; 

std::ofstream create_file(const rclcpp::Logger& LOGGER ){
    const char* home = std::getenv("HOME");
    if (!home) {
        RCLCPP_ERROR(LOGGER, "HOME environment variable not set");
        return {};
    }
    std::filesystem::path dir( std::filesystem::path(home) / "Projects/Masterarbeit/simdata");
    std::error_code ec; 
    std::filesystem::create_directories(dir, ec);
    if (ec) {
        RCLCPP_ERROR(
            LOGGER,
            "Failed to create %s: %s",
            dir.c_str(),
            ec.message().c_str()
        );
        return {};
    }
    std::filesystem::path filepath (dir / "data.json");
    std::ofstream file(filepath);
    if(!file.is_open()){
        RCLCPP_ERROR(
            LOGGER,
            "Failed to open: %s",
            filepath.c_str()
        );
        return {}; 
    }
    RCLCPP_INFO(LOGGER, "Output file opened");
    return file; 
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto move_group_node = rclcpp::Node::make_shared("Fullsim_Node", node_options);
    const auto LOGGER = move_group_node->get_logger(); 
    // We spin up a SingleThreadedExecutor for the current state monitor to get information
    // about the robot's state.
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(move_group_node);
    std::thread([&executor]() { executor.spin(); }).detach();


    std::string urdf;
    {
    std::ifstream urdf_file("model3.urdf");
    urdf.assign((std::istreambuf_iterator<char>(urdf_file)),
                std::istreambuf_iterator<char>());
    }

    move_group_node->declare_parameter("robot_description", urdf);



    static const std::string PLANNING_GROUP = "arm";
    moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Raw pointers are frequently used to refer to the planning group for improved performance.
    const moveit::core::JointModelGroup* joint_model_group =
        move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    // Visualization
    // ^^^^^^^^^^^^^
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools(move_group_node, "model1_right_torso", rviz_visual_tools::RVIZ_MARKER_TOPIC, //this name is topic in RVIZ
                                                        move_group.getRobotModel());

    visual_tools.deleteAllMarkers();

    /* Remote control is an introspection tool that allows users to step through a high level script */
    /* via buttons and keyboard shortcuts in RViz */
    visual_tools.loadRemoteControl();

    // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.0;
    visual_tools.publishText(text_pose, "MoveGroupInterface_Demo", rvt::WHITE, rvt::XLARGE);

    // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
    visual_tools.trigger();

     // Getting Basic Information
    // ^^^^^^^^^^^^^^^^^^^^^^^^^
    //
    // We can print the name of the reference frame for this robot.
    RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());

    // We can also print the name of the end-effector link for this group.
    RCLCPP_INFO(LOGGER, "End effector link: %s", move_group.getEndEffectorLink().c_str());

    // We can get a list of all the groups in the robot:
    // RCLCPP_INFO(LOGGER, "Available Planning Groups:");
    // std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
    //         std::ostream_iterator<std::string>(std::cout, ", "));

    std::vector<Datapoint> data;
    std::shared_mutex mu; 
    int i=0; 
    std::atomic<bool> started_receiving_data=false;
    std::atomic<bool> timer_done=false;
    rclcpp::TimerBase::SharedPtr receivingTimer;
    RCLCPP_INFO(LOGGER, "Waiting for Position Data from Parser ...");
    auto sub_callback = [&data, &mu, &i, &LOGGER, &started_receiving_data, &timer_done, &receivingTimer, &move_group_node](Datapoint::SharedPtr msg){
        started_receiving_data=true; 
        std::unique_lock<std::shared_mutex> lock(mu);
        data.push_back(*msg);
        
        RCLCPP_INFO(LOGGER, "Received packet number: %d", ++i);
        
        if (!started_receiving_data.exchange(true)) {

            receivingTimer = move_group_node->create_wall_timer(
                std::chrono::seconds(2),
                [&]() {
                RCLCPP_INFO(LOGGER, "Collection window ended");

                receivingTimer.reset();   // stop timer
                timer_done.store(true);   // signal main thread
                });
            }
    };
    
    auto _data_subsriber = move_group_node->create_subscription<Datapoint>("datapoint", 100, sub_callback);


    
    while (!timer_done.load() && rclcpp::ok()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }


    // Start the demo
    // ^^^^^^^^^^^^^^^^^^^^^^^^^
    // Planning to a Pose goal
    // ^^^^^^^^^^^^^^^^^^^^^^^
    // We can plan a motion for this group to a desired pose for the
    // end-effector.
    geometry_msgs::msg::Pose target_pose1;
    target_pose1.orientation.w = 1.0;
    target_pose1.position.x = 0.28;
    target_pose1.position.y = -0.2;
    target_pose1.position.z = 0.5;
    move_group.setPoseTarget(target_pose1);

    // Now, we call the planner to compute the plan and visualize it.
    // Note that we are just planning, not asking move_group
    // to actually move the robot.
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    RCLCPP_INFO(LOGGER, "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    move_group.move();


     // Visualizing plans
    // ^^^^^^^^^^^^^^^^^
    // We can also visualize the plan as a line with markers in RViz.
    RCLCPP_INFO(LOGGER, "Visualizing plan 1 as trajectory line");
    visual_tools.publishAxisLabeled(target_pose1, "pose1");
    visual_tools.publishText(text_pose, "Pose_Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(my_plan.trajectory, joint_model_group);
    visual_tools.trigger();
    
    /*
        Creating the Path towards the desired output folder:
    */

    std::ofstream outputfile = create_file(LOGGER); 

    json outputdata; 



    /*
    Printing the joint values to the console:
    */
    const std::vector< std::string > &  jointnames = move_group.getJointNames();
    std::vector<double> jointstates = move_group.getCurrentJointValues();
    for(size_t i=0; i< jointnames.size(); ++i){
        RCLCPP_INFO(LOGGER, "joint state %s: %f", jointnames[i].c_str(), jointstates[i]); 
        outputdata[jointnames[i].c_str()] =  jointstates[i]; 
    }

    outputfile << std::setw(4) <<outputdata <<std::endl; 
    
    using namespace std::chrono_literals;
    std::this_thread::sleep_for(5s); 
    // END_TUTORIAL
    visual_tools.deleteAllMarkers();
    visual_tools.trigger();

    rclcpp::shutdown();
    return 0;
}




