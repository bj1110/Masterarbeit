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

#include "ik_processing/helpers.hpp"

// for convenience
using json = nlohmann::json;
using Datapoint = ik_processing::msg::Datapoint; 
using Data = ik_processing::srv::Data; 


int main(int argc, char** argv)
{
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

    
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Raw pointers are frequently used to refer to the planning group for improved performance.
    const moveit::core::JointModelGroup* joint_model_group =
        move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);


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
        Creating the Path towards the desired output folder:
    */

    std::ofstream outputfile = helpers::create_file(LOGGER); 

    json outputdata; 


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
        Setting Startstate & remember Position before initial Movement to store in output file 
    */

    move_group.setEndEffectorLink("can");
    move_group.setStartStateToCurrentState(); 
    geometry_msgs::msg::Pose startPos = move_group.getCurrentPose().pose; 
    RCLCPP_INFO(LOGGER, "Endeffector link: %s", move_group.getEndEffectorLink().c_str()); 

    /*
        Move to Position 1 
        Position 1 lies approx. x= -25 & y=-220       
    */

    std::map<std::string, double> states_values;
    if (joint_model_group->getVariableDefaultPositions("Pos1", states_values)){
        RCLCPP_INFO(LOGGER, "Moving to default position \"Pos1\" ");
        move_group.setJointValueTarget(states_values); 
        move_group.move(); 
    }else{
        RCLCPP_WARN(LOGGER, "Default state not found."); 
    }



    Datapoint dp1 = data[0];

    /*
        Setting orientation
    */
    // float roll = -2.47;
    // float pitch = 0.0; 
    // float yaw = -1.57;
    tf2::Quaternion q {-0.23278, -0.69127,-0.23118, 0.64383};
    // q.setRPY(roll, pitch , yaw); 
    q.normalize();

    /*
        Offset dataset -> model
    */

    float x_offset= 220 + 250;
    float y_offset=0;
    float z_offset=690;

    /*
        Set Values for planning and moving
    */

    move_group.setGoalOrientationTolerance(2*M_PI);
    move_group.setMaxAccelerationScalingFactor(1.0);
    move_group.setMaxVelocityScalingFactor(1.0);
    
    RCLCPP_INFO(LOGGER, "The tolerance for the Position per default is: %f", move_group.getGoalPositionTolerance());  

    /*
        Aufgrund der Achsenwahl: tausche x und y 
    */
    
    std::vector<geometry_msgs::msg::Pose> waypoints;
    int cnt=0; 
    for(auto dp: data){
        geometry_msgs::msg::Pose p;
        p.position.x = (dp.y1 +x_offset)/1000;
        p.position.y = (dp.x1 +y_offset)/1000;
        p.position.z = (dp.z1 +z_offset)/1000;
        p.orientation =tf2::toMsg(q); 
        waypoints.push_back(p);
    }

    int numPoints= waypoints.size();
    int currPoint=0;
    for(const auto& wp: waypoints){
        move_group.setPoseTarget(wp);
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;

        bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

        RCLCPP_INFO(LOGGER, "Visualizing plan %d/%d (pose goal) %s", ++currPoint, numPoints,  success ? "" : "FAILED");

        move_group.move();

        helpers::store(outputdata, move_group, currPoint); 
    }



    outputfile << std::setw(4) <<outputdata <<std::endl; 
    
    rclcpp::shutdown();
    return 0;
}

