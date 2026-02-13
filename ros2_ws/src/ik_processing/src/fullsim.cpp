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

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/robot_state/cartesian_interpolator.hpp>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.hpp>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <tf2_eigen/tf2_eigen.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit/robot_state/conversions.hpp>

#include <cmath>

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

    /*
        Set Values for planning and moving
    */

    move_group.setGoalOrientationTolerance(0.1);
    move_group.setMaxAccelerationScalingFactor(1.0);
    move_group.setMaxVelocityScalingFactor(1.0);

    /*
        Move to Position 1 
        Position 1 lies approx. x= -25 & y=-220       
    */

    std::map<std::string, double> states_values;
    if (joint_model_group->getVariableDefaultPositions("Pos1_right", states_values)){
        RCLCPP_INFO(LOGGER, "Moving to default position \"Pos1\" ");
        move_group.setJointValueTarget(states_values); 
        move_group.move(); 
    }else{
        RCLCPP_WARN(LOGGER, "Default state not found."); 
    }
    outputdata = move_group.getCurrentPose().pose; 

    /*
        Get the orientation from the preprogrammd Position where the orientation is as desired. 
    */
    tf2::Quaternion q;
    tf2::fromMsg(move_group.getCurrentPose().pose.orientation, q); 

    /*
        Setting orientation
    */
    // float roll = -2.47;
    // float pitch = 0.0; 
    // float yaw = -1.57;
    // tf2::Quaternion q {0.03085, -0.751833,0.143234, 0.64288};
    // q.setRPY(roll, pitch , yaw); 
    // q.normalize();

    /*
        Offset dataset -> model
    */

    float x_offset= 220 + 250;
    float y_offset=0;
    float z_offset=690;



    /*
        Aufgrund der Achsenwahl: tausche x und y 
        Aufgrund Draufsicht: invertiere y 
    */
    
    std::vector<geometry_msgs::msg::Pose> waypoints;
    for(auto dp: data){
        geometry_msgs::msg::Pose p;
        p.position.x = (dp.y1 +x_offset)/1000;
        p.position.y = -(dp.x1 +y_offset)/1000;
        p.position.z = (dp.z1 +z_offset)/1000;
        p.orientation =tf2::toMsg(q); 
        waypoints.push_back(p);
    }


    /*
        Path constraints:
        - can must stay upright 
        - hips must move as little as possible
    */
    moveit_msgs::msg::Constraints path_constraints;

    moveit_msgs::msg::OrientationConstraint oc;
    oc.link_name = "can";
    oc.header.frame_id = move_group.getPlanningFrame();
    oc.orientation = tf2::toMsg(q);
    oc.absolute_x_axis_tolerance = 0.1;  
    oc.absolute_y_axis_tolerance = 0.1;  
    oc.absolute_z_axis_tolerance = 0.1;  
    oc.weight = 1.0;

    path_constraints.orientation_constraints.push_back(oc);

    moveit_msgs::msg::JointConstraint jc;
    jc.joint_name="calumna__joint";
    double curr_columna = move_group.getCurrentState()->getVariablePosition("columna__joint");
    jc.position= curr_columna;
    jc.tolerance_above=0.1;
    jc.tolerance_below=0.1;
    jc.weight=5.0;

    path_constraints.joint_constraints.push_back(jc);

    move_group.setPathConstraints(path_constraints);



    // int numPoints= waypoints.size();
    // int currPoint=0;
    // int num_successes=0;
    // std::vector<moveit::planning_interface::MoveGroupInterface::Plan> plans; 
    // for(const auto& wp: waypoints){
    //     move_group.setPoseTarget(wp);
    //     /*
    //         Set allowed deviance from the goal to the error from the file 
    //     */
    //     move_group.setGoalPositionTolerance(data[currPoint].error1 / 1000);

    //     moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    //     bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    //     RCLCPP_INFO(LOGGER, "Visualizing plan %d/%d (pose goal) %s", ++currPoint, numPoints,  success ? "" : "FAILED");

    //     plans.push_back(my_plan); 
    //     move_group.execute(my_plan);

    //     if(success){
    //         ++num_successes;
    //         helpers::store(outputdata, move_group, currPoint); 
    //     }

    // }
    // RCLCPP_INFO(LOGGER, "The number of successfully computed positions is: %d / %d", num_successes ,numPoints); 

    // move_group.setJointValueTarget(states_values); 
    // move_group.move(); 
    // for(const auto& p: plans){
    //     move_group.execute(p); 
    // }


    // move_group.setPoseTarget(waypoints[0]);
    // moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    // bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    // RCLCPP_INFO(LOGGER, "Moving to the first Position in path, %d", success);
    // move_group.execute(my_plan);
    // helpers::store(outputdata, move_group, 1);

    /*
        custom cost function
        steps:
        1. Make sure the startseed is the same (Position 1 for now)
        2. define the targets
        3. define options
        4. define l2 norm
        5. define custom cost callbackfn
        6. call low-lvl planner
        7. add time to each calculated waypoint
        8. build and execute the Robot Trajecotry
    */

    auto robot_model = move_group.getRobotModel();
    moveit::core::RobotState rs (robot_model);
    rs.setToDefaultValues(
        robot_model->getJointModelGroup("arm"),
        "Pos1_right"
    );
    moveit_msgs::msg::RobotState state_msg;
    moveit::core::robotStateToRobotStateMsg(rs, state_msg);
    move_group.setStartStateToCurrentState(); 
    
    /* 2: */
    Eigen::Isometry3d target;
    EigenSTL::vector_Isometry3d targets;
    for(const auto& wp: waypoints){
        tf2::fromMsg(wp, target);
        targets.push_back(target);
    }

    /* 3: */
    moveit::core::GroupStateValidityCallbackFn callback_fn;
    kinematics::KinematicsQueryOptions opts;
    opts.return_approximate_solution = true;
    auto start_state = move_group.getCurrentState();
    std::vector<moveit::core::RobotStatePtr> traj;
    moveit::core::MaxEEFStep max_eef_step(0.02, 0.2);
    moveit::core::CartesianPrecision cartesian_precision{ .translational = 0.005,
                                                        .rotational = 0.05,
                                                        .max_resolution = 5e-3 };

    /* 4: */
    const auto compute_l2_norm = [](std::vector<double> solution, std::vector<double> start) {
        double sum = 0.0;
        for (size_t ji = 0; ji < solution.size(); ji++)
        {
            double d = solution[ji] - start[ji];
            sum += d * d;
        }
        return sum;
    };

    const size_t hip_idx = joint_model_group->getVariableGroupIndex("columna__joint");
    const auto penalize_hip = [&hip_idx](std::vector<double>solution, double delta=0.0005, double penalty_size= 1.0, double penalty_gradient=1.0){
        double d= solution[hip_idx]; 
        double step = 0.5 * (1+ std::tanh(penalty_gradient * (d-delta))); 
        double penalty = penalty_size * step* (d-delta); 
        return penalty; 
    };

    const moveit::core::JointModel* elbow_model = robot_model->getJointModel("elbow_yaw_joint");
    const moveit::core::VariableBounds elbowBounds = (elbow_model-> getVariableBounds())[0];
    const double elbow_max = elbowBounds.max_position_; /*maximum bend*/ 
    const size_t elbow_idx = joint_model_group->getVariableGroupIndex("elbow_yaw_joint");
    const auto incentivise_elbow = [&elbow_idx, &elbow_max] (std::vector<double>solution){
        double d= elbow_max - solution[elbow_idx]; 
        return -(d*d) ; 
    };

    /* 5: */
    const double hip_weight = 0.0009;
    const double elbow_weight= 0.000000001; 
    const std::vector<std::string> joint_model_names = joint_model_group->getJointModelNames();
    const auto cost_fn = [&hip_weight, &elbow_weight, &compute_l2_norm, &penalize_hip, &incentivise_elbow /*, &LOGGER*/]
                                                (const geometry_msgs::msg::Pose& /*goal_pose*/,
                                                const moveit::core::RobotState& solution_state,
                                                moveit::core::JointModelGroup const* jmg,
                                                const std::vector<double>& /*seed_state*/) {
        std::vector<double> proposed_joint_positions;
        solution_state.copyJointGroupPositions(jmg, proposed_joint_positions);
        //double abs = compute_l2_norm(proposed_joint_positions, seed_state);
        double hip = hip_weight* penalize_hip(proposed_joint_positions);
        double elbow = elbow_weight* incentivise_elbow(proposed_joint_positions);
        
        return hip + elbow ;
    };
    
    /* 6: */
    const auto frac = moveit::core::CartesianInterpolator::computeCartesianPath(
        start_state.get(), joint_model_group, traj, joint_model_group->getLinkModel("can"), targets, true,
        max_eef_step, cartesian_precision, callback_fn, opts , cost_fn);

    RCLCPP_INFO(LOGGER, "\033[32mComputed %f percent of cartesian path.\033[0m", frac.value * 100.0);

    /* 7: */
    robot_trajectory::RobotTrajectory rt(start_state->getRobotModel(), PLANNING_GROUP);
    for (const moveit::core::RobotStatePtr& traj_state : traj)
    rt.addSuffixWayPoint(traj_state, 0.0);
    trajectory_processing::TimeOptimalTrajectoryGeneration time_param;
    time_param.computeTimeStamps(rt, 1.0);

    /* 8: */
    moveit_msgs::msg::RobotTrajectory rt_msg;
    rt.getRobotTrajectoryMsg(rt_msg);
    move_group.execute(rt_msg);
    outputdata = rt_msg; 

    if(frac.value<1.0){
        RCLCPP_INFO(LOGGER, "Path was not computed fully, attempting to move the last part"); 
        move_group.setPoseTarget(waypoints.back());
        move_group.clearPathConstraints(); 
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        RCLCPP_INFO(LOGGER, "Computation of missing path %s", success? "successful": "\033[31mFAILURE\033[0m");
        if(success){
            move_group.execute(my_plan); 
        }
        else{
            size_t pt = waypoints.size()-2;
            while(pt>0){
                move_group.setPoseTarget(waypoints[pt]);
                moveit::planning_interface::MoveGroupInterface::Plan my_plan;
                bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
                if(success){
                    int num_points = waypoints.size(); 
                    int num_points_not_movedto = num_points-pt;
                    float percentage_reachable = ( (float) pt/(float) num_points);
                    RCLCPP_INFO(LOGGER, "\033[32mTotal movement: %f, with %d/%d Points not reachable \033[0m",percentage_reachable, num_points_not_movedto, num_points);
                    move_group.execute(my_plan);
                    break;
                }
                --pt; 
            }

        }
    }
    // moveit_msgs::msg::RobotTrajectory trajectory;

    // const double eef_step = 0.001;
    // const bool avoid_collisions = true;

    // std::vector<geometry_msgs::msg::Pose> cartWaypoints;

    // for(size_t i=0; i< waypoints.size(); ++i){
    //     if(i%3==0){
    //         cartWaypoints.push_back(waypoints[i]);
    //     }
    // }

    // double fraction = move_group.computeCartesianPath(
    //     cartWaypoints,
    //     eef_step,
    //     trajectory,
    //     path_constraints, 
    //     avoid_collisions
    // );

    // move_group.execute(trajectory);
    // RCLCPP_INFO(LOGGER, "Cartesian path fraction: %.2f", fraction);

    // outputdata = trajectory; 

    // move_group.setPoseTarget(waypoints[waypoints.size()-1]);
    // move_group.clearPathConstraints(); 
    // moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    // move_group.plan(my_plan);


    outputfile << std::setw(4) <<outputdata <<std::endl; 
    
    rclcpp::shutdown();
    return 0;
}

