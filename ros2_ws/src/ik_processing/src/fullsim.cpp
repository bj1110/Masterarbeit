#include "nullspace_solver/solver.hpp"


#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

// #include <moveit_visual_tools/moveit_visual_tools.hpp>
#include <chrono>
#include <thread>

#include <filesystem>
#include <stdexcept>

#include <nlohmann/json.hpp>
#include "ik_processing/msg/datapoint.hpp"
#include "ik_processing/msg/header.hpp"
#include "ik_processing/srv/data.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "ik_processing/helpers.hpp"
#include "ik_processing/evaluator.hpp"
#include "ik_processing/rigid_body_algorithm.hpp"

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/robot_state/cartesian_interpolator.hpp>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.hpp>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <tf2_eigen/tf2_eigen.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit/robot_state/conversions.hpp>

#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/robot_state/conversions.hpp>
#include <moveit/planning_pipeline/planning_pipeline.hpp>
#include <moveit/planning_interface/planning_interface.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.hpp>
#include <moveit/kinematic_constraints/utils.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>



#include <cmath>

// for convenience
using json = nlohmann::json;
using Datapoint = ik_processing::msg::Datapoint; 
using Header = ik_processing::msg::Header; 
using Data = ik_processing::srv::Data; 
using crb = rigid_body_algorithm::crb; 


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto move_group_node = rclcpp::Node::make_shared("Fullsim_Node", node_options);
    const auto LOGGER = move_group_node->get_logger(); 

    bool recalculate = move_group_node->get_parameter("recalculate").as_bool();
    std::string urdf_string = move_group_node->get_parameter("urdf").as_string(); 


    static const std::string PLANNING_GROUP = "arm";
    moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);

    // We spin up a SingleThreadedExecutor for the current state monitor to get information
    // about the robot's state.
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(move_group_node);
    std::thread([&executor]() { executor.spin(); }).detach();
   
    robot_model_loader::RobotModelLoader robot_model_loader(move_group_node);
    const moveit::core::RobotModelPtr robot_model = robot_model_loader.getModel();

    // Raw pointers are frequently used to refer to the planning group for improved performance.
    const moveit::core::JointModelGroup* joint_model_group =
        move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);


    /*
        Get the Data from the Parser
    */
    auto client = move_group_node->create_client<Data>("parse_data");
    while(!client->wait_for_service(std::chrono::seconds(1))){
        if(!rclcpp::ok()){
            RCLCPP_INFO(LOGGER, "Client interupted while waiting for service to be available");
            rclcpp::shutdown();
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
        rclcpp::shutdown();
        return 1;
    }

    auto result = result_future.get();
    Header header = result->header;
    [[maybe_unused]] int numElements = header.len;
    std::vector<Datapoint> data = result->data; 

    RCLCPP_INFO(LOGGER, "Data received from Parser");

    /*
        Move to approx. Startposition     
    */

    auto current_state = move_group.getCurrentState();
    current_state->enforceBounds();

    move_group.setStartState(*current_state);
    // move_group.setStartStateToCurrentState(); 
    std::string startpos_name = helpers::get_startpos(header);
    std::map<std::string, double> states_values;
    if (joint_model_group->getVariableDefaultPositions(startpos_name, states_values)){
        RCLCPP_INFO(LOGGER, "Moving to approx. Startposition \"%s\" ", startpos_name.c_str());
        move_group.setJointValueTarget(states_values); 
        move_group.move(); 
        move_group.setStartStateToCurrentState(); 

        // const geometry_msgs::msg::Pose initial_pose = move_group.getCurrentPose().pose;
        //RCLCPP_INFO(LOGGER, "\033[34m initial pose: \n %s \033[0m", helpers::print_pose(initial_pose).c_str()); 
    }else{
        RCLCPP_WARN(LOGGER, "\033[31mDefault state not found.\033[0m"); 
    }


    /*
        Check if just the playout of already calculated data is wanted.
        Only header data is needed for that.
    */
    if(!recalculate){
        if(std::filesystem::path filepath = helpers::create_datapath(LOGGER, header); 
        !filepath.empty() && std::filesystem::exists(filepath)){
            RCLCPP_INFO(LOGGER, "This has been calculated before, loading..."); 
            std::ifstream inputfile (filepath); 
            if(!inputfile.is_open()){
                RCLCPP_ERROR(LOGGER, "\033[34mFile could not be opened\033[0m"); 
                rclcpp::shutdown();
                return 1;
            }
            json inputdata = json::parse(inputfile);
            moveit_msgs::msg::RobotTrajectory rt_msg;
            from_json(inputdata, rt_msg);
            rt_msg.joint_trajectory.header.stamp = move_group_node->now();
            rt_msg.joint_trajectory.header.frame_id = move_group.getPlanningFrame();
            RCLCPP_INFO(LOGGER, "Loading complete, displaying path");
            move_group.execute(rt_msg);
            rclcpp::shutdown(); 
            return 0;
        }
    }


    /*
        create vector of the timestamps
        fill later.
    */
    std::vector<double> timesteps; 

        
    /*
        Creating the Path towards the desired output folder:
    */

    std::ofstream outputfile = helpers::create_file(LOGGER, header); 
    json outputdata; 


    /*
        Setting Startstate
    */

    move_group.setEndEffectorLink("can");
    move_group.setStartStateToCurrentState(); 

    /*
        Set Values for planning and moving
    */

    move_group.setGoalOrientationTolerance(0.1);
    move_group.setMaxAccelerationScalingFactor(1.0);
    move_group.setMaxVelocityScalingFactor(1.0);


    /*
        Get the orientation from the preprogrammd Position where the orientation is as desired. 
    */
    tf2::Quaternion q;
    tf2::fromMsg(move_group.getCurrentPose().pose.orientation, q); 


    /*
        Offset dataset -> model
        1. x value should start about 25cm infront of robot center (15cm table->Point1; 10cm body->table)
        2. y value -
        3. z value should be raised to about the height where the forearm is parallel to the ground in Pos1

        4. In case of Agent2 movement: mirror x and y
    */
    [[maybe_unused]] const float AGENT_SIGN = (header.is_agent1)? 1 : -1;
    const float TABLE_PERSON_DIST = 100;
    const float TABLE_MARKER_DIST = 150;
    const float SPINE_HEIGHT = 477;
    const float CHAIR_TABLE_HEIGHTDIFF = 330;
    const float CAN_HEIGHT = 120; 
    [[maybe_unused]] const float PREV_HEIGHT = 690; 
    const float X_OFFSET = 220 + TABLE_PERSON_DIST + TABLE_MARKER_DIST;
    const float Y_OFFSET = 0;
    const float Z_OFFSET = SPINE_HEIGHT + CHAIR_TABLE_HEIGHTDIFF -CAN_HEIGHT;


    const Eigen::Vector3f agent1_av_pos1 {-241.88, 2.38, 0};
    const Eigen::Vector3f agent1_av_pos5 {237.46, 4.87, 0};
    const Eigen::Vector3f agent2_av_pos1 {-238.43, -8.01, 0};
    const Eigen::Vector3f agent2_av_pos5 {237.0, -3.39, 0};

    const Eigen::Vector3f agent1_av_path_1_5 = agent1_av_pos5 - agent1_av_pos1;
    const Eigen::Vector3f agent5_av_path_5_1 = agent2_av_pos1 - agent2_av_pos5;    

    const auto crossproduct = agent1_av_path_1_5.cross(agent5_av_path_5_1);
    const auto dotproduct = agent1_av_path_1_5.dot(agent5_av_path_5_1);
    const auto abs_agent1_path = agent1_av_path_1_5.norm();
    const auto abs_agent2_path = agent5_av_path_5_1.norm();

    const auto abs_both_paths =abs_agent1_path * abs_agent2_path;
    const auto cos_phi = dotproduct / abs_both_paths;
    const auto sin_phi = crossproduct.z() / abs_both_paths; 


    Eigen::Matrix3f rotationsmatrix {
        {cos_phi, -sin_phi, 0.0},
        {sin_phi, cos_phi, 0.0}, 
        {0.0, 0.0, 1.0}  
    };



    /*
        Due to the axis choice in model compared to data: swap x and y. 
        Due to the top view: invert y.
        Chosing either agent1 or agent2 to compute the path
    */
    std::vector<geometry_msgs::msg::Pose> waypoints;
    for(auto dp: data){
        Eigen::Vector3f point {dp.y1, dp.x1, dp.z1}; 
        if(!header.is_agent1){
            point.x() = dp.y2;
            point.y() = dp.x2;
            point.z() = dp.z2;
            point = rotationsmatrix * point;
        }
        geometry_msgs::msg::Pose p;
        p.position.x = (point.x() +X_OFFSET)/1000;
        p.position.y = -(point.y() +Y_OFFSET)/1000;
        p.position.z = (point.z() +Z_OFFSET)/1000;
        p.orientation =tf2::toMsg(q); 
        waypoints.push_back(p);
        timesteps.push_back(dp.time);
    }


    // RCLCPP_INFO(LOGGER, "\033[34mFirst point: \n %s\033[0m", helpers::print_pose(waypoints.front()).c_str()); 
    // RCLCPP_INFO(LOGGER, "\033[34mLast point: \n %s \033[0m", helpers::print_pose(waypoints.back()).c_str()); 


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
    
    /*
        Set startstate from the default state states_values
    */
   
    std::vector<double> joint_values;
    for(const std::string& name: variable_names){
        const double value = states_values[name];
        joint_values.push_back(value);
    }
    Eigen::VectorXd start_configuration = Eigen::Map<Eigen::VectorXd>(joint_values.data(), dof);
    
    std::string ee_frame= "can";

    nullspace_solver::Solver solver {urdf_string, ee_frame, waypoints, timesteps, LOGGER}; 

    solver.setJointLimits(q_min, q_max); 

    moveit_msgs::msg::RobotTrajectory trajectory;
    

    //TODO: ist start_configuration in der richtigen reihenfolge???
 
    bool ik_ok= solver.solve(start_configuration, trajectory); 

    if(ik_ok){
        move_group.execute(trajectory); 
        outputdata=trajectory; 
        outputdata["info"]["NJS"] = evaluator::calculate_av_NJS(trajectory, LOGGER); 
    }else{
        RCLCPP_INFO(LOGGER, "\033[31m Solver failed to converge\033[0m"); 
    }

    const geometry_msgs::msg::Pose ee_final_pose = move_group.getCurrentPose().pose;
    const geometry_msgs::msg::Pose ee_target_pose = waypoints.back(); 
    RCLCPP_INFO(LOGGER, "\033[32m Actual pose: \n %s \033[0m", helpers::print_pose(ee_final_pose).c_str());
    RCLCPP_INFO(LOGGER, "\033[34m Target pose: \n %s \033[0m", helpers::print_pose(ee_target_pose).c_str());


    auto input_path_eval = evaluator::endeffector(waypoints, timesteps, LOGGER);
    outputdata["info"]["Endeffector_NJS"] = input_path_eval.get_NJS(); 

    outputfile << std::setw(4) <<outputdata <<std::endl; 

    rclcpp::shutdown();
    return 0;
}

