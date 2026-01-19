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
#include "ik_processing/srv/data.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// for convenience
using json = nlohmann::json;
using Datapoint = ik_processing::msg::Datapoint; 
using Data = ik_processing::srv::Data; 

namespace geometry_msgs::msg
{
    inline void to_json(nlohmann::json& j, const Pose& p)
    {
        j = {
            {"position", {
                {"x", p.position.x},
                {"y", p.position.y},
                {"z", p.position.z}
            }},
            {"orientation", {
                {"x", p.orientation.x},
                {"y", p.orientation.y},
                {"z", p.orientation.z},
                {"w", p.orientation.w}
            }}
        };
    }

    inline void from_json(const nlohmann::json& j, Pose& p)
    {
        j.at("position").at("x").get_to(p.position.x);
        j.at("position").at("y").get_to(p.position.y);
        j.at("position").at("z").get_to(p.position.z);

        j.at("orientation").at("x").get_to(p.orientation.x);
        j.at("orientation").at("y").get_to(p.orientation.y);
        j.at("orientation").at("z").get_to(p.orientation.z);
        j.at("orientation").at("w").get_to(p.orientation.w);
    }
}

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
    // std::this_thread::sleep_for(std::chrono::seconds(20)); 
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto move_group_node = rclcpp::Node::make_shared("Fullsim_Node", node_options);
    const auto LOGGER = move_group_node->get_logger(); 

    static const std::string PLANNING_GROUP = "arm";
    moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);

    // We spin up a SingleThreadedExecutor for the current state monitor to get information
    // about the robot's state.
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(move_group_node);
    std::thread([&executor]() { executor.spin(); }).detach();


    // std::string urdf;
    // {
    // std::ifstream urdf_file("model3.urdf");
    // urdf.assign((std::istreambuf_iterator<char>(urdf_file)),
    //             std::istreambuf_iterator<char>());
    // }

    // move_group_node->declare_parameter("robot_description", urdf);
    

    
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Raw pointers are frequently used to refer to the planning group for improved performance.
    const moveit::core::JointModelGroup* joint_model_group =
        move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    // Visualization
    // ^^^^^^^^^^^^^
    //namespace rvt = rviz_visual_tools;
    //moveit_visual_tools::MoveItVisualTools visual_tools(move_group_node, "model1_right_torso", rviz_visual_tools::RVIZ_MARKER_TOPIC, //this name is topic in RVIZ
    //                                                    move_group.getRobotModel());

    //visual_tools.deleteAllMarkers();

    /* Remote control is an introspection tool that allows users to step through a high level script */
    /* via buttons and keyboard shortcuts in RViz */
    //visual_tools.loadRemoteControl();

    // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
    //Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    //text_pose.translation().z() = 1.0;
    //visual_tools.publishText(text_pose, "MoveGroupInterface_Demo", rvt::WHITE, rvt::XLARGE);

    // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
    //visual_tools.trigger();

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

    auto client = move_group_node->create_client<Data>("parse_data");
    while(!client->wait_for_service(std::chrono::seconds(1))){
        if(!rclcpp::ok()){
            RCLCPP_INFO(LOGGER, "Client interupted while waiting for service to be available");
            return 1;
        }
        RCLCPP_INFO(LOGGER, "Waiting for data...");
    }
    auto request = std::make_shared<Data::Request>();
    request->str = "Req";
    auto result_future = client->async_send_request(request);
    if (result_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready)
    {
        RCLCPP_ERROR(LOGGER, "Service call timed out");
        return 1;
    }

    auto result = result_future.get();
    [[maybe_unused]] int numElements = result->len;
    std::vector<Datapoint> data = result->data; 

    RCLCPP_INFO(LOGGER, "Data received from Parser");

    /*
        Der Tisch hat 80cm Durchmesser.
        Die Punkte sind je 15cm vom Rand weg. 
        Punkt 9 liegt in der Mitte.
        Dh. Punkt 9 liegt bei mindestens 25cm vor dem Bauch :D -> Nehmen wir die 15cm und sagen bei 40cm
        Der Arm kann gerade so die 65cm erreichen. 
        Frage: Welche Höhe sollte das sein? möglichst hoch ist praktisch... 
        In den Daten: erster Wert lateral, zweiter vert ventral, dritter wert kranial
            d.h. ich muss die ersten 2 Werte vertauschen. 
        Koordinaten für die Position 9 scheinen sehr verschieden zu sein... 

    */

    /*
        der Punkt 1 liegt bei ca x= -25 und y=-220
    */

    std::map<std::string, double> states_values;
    if (joint_model_group->getVariableDefaultPositions("Pos1", states_values)){
        RCLCPP_INFO(LOGGER, "Moving to default position \"Pos1\" ");
        move_group.setJointValueTarget(states_values); 
        move_group.move(); 
    }else{
        RCLCPP_WARN(LOGGER, "Default state not found."); 
    }


    // Start the demo
    // ^^^^^^^^^^^^^^^^^^^^^^^^^
    // Planning to a Pose goal
    // ^^^^^^^^^^^^^^^^^^^^^^^
    // We can plan a motion for this group to a desired pose for the
    // end-effector.

    //remember starting position:
    geometry_msgs::msg::Pose startPos = move_group.getCurrentPose().pose; 
    move_group.setEndEffectorLink("can");
    move_group.setStartStateToCurrentState(); 

    Datapoint dp1 = data[0];

    float roll = -2.47;
    float pitch = 0.0; 
    float yaw = -1.57;
    tf2::Quaternion q {-0.23278, -0.69127,-0.23118, 0.64383};
    // q.setRPY(roll, pitch , yaw);  // roll, pitch, yaw
    q.normalize();



    float x_offset= 220 + 250;
    float y_offset=0;
    float z_offset=690;

    RCLCPP_INFO(LOGGER, "First Dataset: t=%f s, x= %f mm, y=%f mm, z=%f mm", dp1.time, dp1.x1, dp1.y1, dp1.z1); 

    // Aufgrund der Achsenwahl: tausche x und y 
    geometry_msgs::msg::Pose target_pose1;
    // target_pose1.orientation.y=-0.69; 
    target_pose1.orientation = tf2::toMsg(q);
    target_pose1.position.x = (dp1.y1 +x_offset )/1000;
    target_pose1.position.y = (dp1.x1 +y_offset)/1000;
    target_pose1.position.z = (dp1.z1 +z_offset)/1000;
    move_group.setPoseTarget(target_pose1);
    
    move_group.setGoalOrientationTolerance(0.01);

    RCLCPP_INFO(LOGGER, "First Target Pose: w=%f, x=%f m, y=%f m , z=%f m ",target_pose1.orientation.w, 
        target_pose1.position.x,  target_pose1.position.y,  target_pose1.position.z);

    // // geometry_msgs::msg::Pose target_pose2;
    // // target_pose2.position.x = 0.28;
    // // target_pose2.position.y = -0.2;
    // // target_pose2.position.z = 0.5;
    // // move_group.setPoseTarget(target_pose2);
    

    // // RCLCPP_INFO(LOGGER, "Second Target Pose: w=%f, x=%f, x=%f, z=%f",target_pose2.orientation.w, 
    // //  target_pose2.position.x,  target_pose2.position.y,  target_pose2.position.z);



    // // Now, we call the planner to compute the plan and visualize it.
    // // Note that we are just planning, not asking move_group
    // // to actually move the robot.
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    RCLCPP_INFO(LOGGER, "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    move_group.move();


     // Visualizing plans
    // ^^^^^^^^^^^^^^^^^
    // We can also visualize the plan as a line with markers in RViz.
    // RCLCPP_INFO(LOGGER, "Visualizing plan 1 as trajectory line");
    // visual_tools.publishAxisLabeled(target_pose1, "pose1");
    // visual_tools.publishText(text_pose, "Pose_Goal", rvt::WHITE, rvt::XLARGE);
    // visual_tools.publishTrajectoryLine(my_plan.trajectory, joint_model_group);
    // visual_tools.trigger();

    // std::vector<geometry_msgs::msg::Pose> waypoints;
    // int cnt=0; 
    // for(auto dp: data){
    //     if(dp==data[1] || dp==data[2]){
    //         continue; 
    //     }
    //     if((++cnt)%5!=0){
    //         continue; 
    //     }
    //     geometry_msgs::msg::Pose p;
    //     p.position.x = (dp.y1 +x_offset)/1000;
    //     p.position.y = (dp.x1 +y_offset)/1000;
    //     p.position.z = (dp.z1 +z_offset)/1000;
    //     waypoints.push_back(p);
    // }

    // const double eef_step = 0.01;
    // moveit_msgs::msg::RobotTrajectory trajectory;
    // moveit_msgs::msg::Constraints path_contraints;
    // const bool avoid_collisions=true; 
    // moveit_msgs::msg::MoveItErrorCodes* errorcode = nullptr; 

    // double fraction = move_group.computeCartesianPath(waypoints, eef_step, trajectory, /*path_contraints, */ avoid_collisions, errorcode);
    // RCLCPP_INFO(LOGGER, "Number of Waypoints: %ld", waypoints.size());
    // RCLCPP_INFO(LOGGER, "Visualizing plan 4 (Cartesian path) (%.2f%% achieved)", fraction * 100.0);
    // if(errorcode && errorcode->val != moveit::core::MoveItErrorCode::SUCCESS){
    //     RCLCPP_INFO(LOGGER, "Error in Carthesian Path: "); 
    //     RCLCPP_INFO(LOGGER, "Error in Carthesian Path: %s", (errorcode->message).c_str());
    // }
    // move_group.execute(trajectory); 
    
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
        // RCLCPP_INFO(LOGGER, "joint state %s: %f", jointnames[i].c_str(), jointstates[i]); 
        outputdata["end_jointstates"][jointnames[i].c_str()] =  jointstates[i]; 
    }

    geometry_msgs::msg::Pose endPos = move_group.getCurrentPose().pose; 
    outputdata["start"]= startPos;
    outputdata["end"]= endPos; 

    outputfile << std::setw(4) <<outputdata <<std::endl; 
    
    // END_TUTORIAL
    //visual_tools.deleteAllMarkers();
    //visual_tools.trigger();

    rclcpp::shutdown();
    return 0;
}

