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
    [[maybe_unused]] bool fully_calc = move_group_node->get_parameter("fully_calc").as_bool();
    std::string urdf_string = move_group_node->get_parameter("urdf").as_string(); 


    static const std::string PLANNING_GROUP = "arm";
    moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);

    // We spin up a SingleThreadedExecutor for the current state monitor to get information
    // about the robot's state.
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(move_group_node);
    std::thread([&executor]() { executor.spin(); }).detach();

    
    robot_model_loader::RobotModelLoaderPtr robotmodelloader(
        std::make_shared<robot_model_loader::RobotModelLoader>(move_group_node, "robot_description")
    );
    planning_scene_monitor::PlanningSceneMonitorPtr psm(
        std::make_shared<planning_scene_monitor::PlanningSceneMonitor> (move_group_node, robotmodelloader)
    );
    psm->startSceneMonitor();
    psm->startWorldGeometryMonitor();
    psm->startStateMonitor();

    moveit::core::RobotModelPtr robot_model = robotmodelloader->getModel(); 

    moveit::core::RobotStatePtr robot_state (
        std::make_shared<moveit::core::RobotState> (planning_scene_monitor::LockedPlanningSceneRO(psm)->getCurrentState())
    );

    planning_pipeline::PlanningPipelinePtr planning_pipeline(
        std::make_shared<planning_pipeline::PlanningPipeline> (robot_model, move_group_node, "stomp")
    );


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

    std::string startpos_name = helpers::get_startpos(header);
    std::map<std::string, double> states_values;
    if (joint_model_group->getVariableDefaultPositions(startpos_name, states_values)){
        RCLCPP_INFO(LOGGER, "Moving to approx. Startposition \"%s\" ", startpos_name.c_str());
        move_group.setJointValueTarget(states_values); 
        move_group.move(); 
        const geometry_msgs::msg::Pose initial_pose = move_group.getCurrentPose().pose;
        RCLCPP_INFO(LOGGER, "\033[31m initial pose: \n %s \033[0m", helpers::print_pose(initial_pose).c_str()); 
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


    RCLCPP_INFO(LOGGER, "\033[34mFirst point: \n %s\033[0m", helpers::print_pose(waypoints.front()).c_str()); 
    RCLCPP_INFO(LOGGER, "\033[34mLast point: \n %s \033[0m", helpers::print_pose(waypoints.back()).c_str()); 


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
    
    // RCLCPP_INFO_STREAM(LOGGER, "start config: "<< start_configuration.transpose()); 

    //TODO: ist start_configuration in der richtigen reihenfolge???
 
    bool ik_ok= solver.solve(start_configuration, trajectory); 

    if(ik_ok){
        move_group.execute(trajectory); 
    }else{
        RCLCPP_INFO(LOGGER, "\033[31m Solver failed to converge\033[0m"); 
    }

    const geometry_msgs::msg::Pose ee_final_pose = move_group.getCurrentPose().pose;
    const geometry_msgs::msg::Pose ee_target_pose = waypoints.back(); 
    RCLCPP_INFO(LOGGER, "\033[32m Actual pose: \n %s \033[0m", helpers::print_pose(ee_final_pose).c_str());
    RCLCPP_INFO(LOGGER, "\033[34m Target pose: \n %s \033[0m", helpers::print_pose(ee_target_pose).c_str());


    rclcpp::shutdown();
    return 0;

    /*
    
        END OF MY CUSTOM SOLVER CALLS 
    
    */


    planning_interface::MotionPlanRequest req;
    req.pipeline_id = "planning_pipeline";
    req.planner_id = "stomp";
    req.allowed_planning_time = 1.0;
    req.max_velocity_scaling_factor = 1.0;
    req.max_acceleration_scaling_factor = 1.0;
    planning_interface::MotionPlanResponse res;
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "base_link";
    pose.pose.position.x = waypoints.back().position.x;
    pose.pose.position.y = waypoints.back().position.y;
    pose.pose.position.z = waypoints.back().position.z;
    pose.pose.orientation = tf2::toMsg(q);

    std::vector<double> tolerance_pose(3, 0.1);
    std::vector<double> tolerance_angle(3, 0.1);

    req.group_name = PLANNING_GROUP;
    moveit_msgs::msg::Constraints pose_goal =
        kinematic_constraints::constructGoalConstraints("can", pose, tolerance_pose, tolerance_angle);
    req.goal_constraints.push_back(pose_goal);

    moveit_msgs::msg::RobotState rs_msg;
    moveit::core::robotStateToRobotStateMsg(*robot_state, rs_msg); 
    req.start_state = rs_msg;  
    {
        planning_scene_monitor::LockedPlanningSceneRO lscene(psm);
        /* Now, call the pipeline and check whether planning was successful. */
        /* Check that the planning was successful */
        if (!planning_pipeline->generatePlan(lscene, req, res) || res.error_code.val != res.error_code.SUCCESS)
        {
            RCLCPP_ERROR(LOGGER, "Could not compute plan successfully");
            rclcpp::shutdown();
            return -1;
        }
        else{
            RCLCPP_INFO(LOGGER, "\033[31m Success \033[0m");
        }
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
    oc.absolute_x_axis_tolerance = 0.1;  //TODO: allow for tolance which would equal the tilting of the hand
    oc.absolute_y_axis_tolerance = 0.1;  
    oc.absolute_z_axis_tolerance = 2*M_PI;  
    oc.weight = 1.0;

    path_constraints.orientation_constraints.push_back(oc);

    // moveit_msgs::msg::JointConstraint jc;
    // jc.joint_name="calumna__joint";
    // double curr_columna = move_group.getCurrentState()->getVariablePosition("columna_flex_joint");
    // jc.position= curr_columna;
    // jc.tolerance_above=0.1;
    // jc.tolerance_below=0.1;
    // jc.weight=5.0;

    // path_constraints.joint_constraints.push_back(jc);

    move_group.setPathConstraints(path_constraints);





    moveit::core::RobotState rs (robot_model);
    rs.setToDefaultValues(
        robot_model->getJointModelGroup("arm"),
        startpos_name
    );
    moveit_msgs::msg::RobotState state_msg;
    moveit::core::robotStateToRobotStateMsg(rs, state_msg);
    move_group.setStartStateToCurrentState(); 


    move_group.setJointValueTarget(waypoints.back()); 

    moveit::planning_interface::MoveGroupInterface::Plan plan;

    bool success = (move_group.plan(plan) ==
                    moveit::core::MoveItErrorCode::SUCCESS);

    if(success){
        move_group.execute(plan);
        RCLCPP_INFO(LOGGER, "\033[37m success with STOMP \033[0m");
    }


    /*
      get the urdf -> links -> interia  
    */
    // const auto urdf = robot_model->getURDF();
    // std::vector<std::shared_ptr<urdf::Link>> urdf_links {};
    // urdf->getLinks(urdf_links);
    // std::unordered_map<std::string, urdf::Inertial> inertias;
    // std::vector<std::string> link_names;
    // for(const auto& link: urdf_links){
    //     std::string link_name= link->name;
    //     urdf::Inertial link_inertial = *(link->inertial);
    //     inertias.insert({link_name, link_inertial}); 
    //     link_names.push_back(link_name);
    // }


    
    // TODO: initialized targets directly from data
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
    moveit::core::MaxEEFStep max_eef_step(0.01, 0.1);
    moveit::core::CartesianPrecision cartesian_precision{ .translational = 0.005,
                                                        .rotational = 0.05,
                                                        .max_resolution = 5e-3 };

    /* 4: */

    
    std::vector<double> joint_states;
    start_state->copyJointGroupPositions(joint_model_group, joint_states);
    crb crb_{ joint_states , urdf_string , LOGGER}; 
    crb_.crba(); 
    Eigen::MatrixXd inertia = crb_.get_inertia_Matrix();

    const double duration = 0.1; 
    const auto compute_energy = [&crb_, &duration](const std::vector<double>& start, const std::vector<double>& solution){
        crb_.update_model_state(solution);
        Eigen::MatrixXd inertia_matrix = crb_.get_inertia_Matrix();
        Eigen::VectorXd start_positions; 
        start_positions = Eigen::Map< const Eigen::VectorXd>(
            start.data(),
            start.size()
        );
        Eigen::VectorXd end_positions;
        end_positions = Eigen::Map<const Eigen::VectorXd>(
            solution.data(),
            solution.size()
        ); 
        Eigen::VectorXd speeds = (end_positions - start_positions) / duration; 
        double T = 0.5 * speeds.transpose() * inertia_matrix * speeds;
        return T; 
    };


    // const auto compute_l2_norm = [](std::vector<double> solution, std::vector<double> start) {
    //     double sum = 0.0;
    //     for (size_t ji = 0; ji < solution.size(); ji++)
    //     {
    //         double d = solution[ji] - start[ji];
    //         sum += d * d;
    //     }
    //     return sum;
    // };

    // const size_t hip_idx = joint_model_group->getVariableGroupIndex("columna_flex_joint");
    // const auto penalize_hip = [&hip_idx](std::vector<double>solution, double delta=0.0005, double penalty_size= 1.0, double penalty_gradient=1.0){
    //     double d= solution[hip_idx]; 
    //     double step = 0.5 * (1+ std::tanh(penalty_gradient * (d-delta))); 
    //     double penalty = penalty_size * step* (d-delta); 
    //     return penalty; 
    // };

    // const moveit::core::JointModel* elbow_model = robot_model->getJointModel("elbow_yaw_joint");
    // const moveit::core::VariableBounds elbowBounds = (elbow_model-> getVariableBounds())[0];
    // const double elbow_max = elbowBounds.max_position_; /*maximum bend*/ 
    // const size_t elbow_idx = joint_model_group->getVariableGroupIndex("elbow_yaw_joint");
    // const auto incentivise_elbow = [&elbow_idx, &elbow_max] (std::vector<double>solution){
    //     double d= elbow_max - solution[elbow_idx]; 
    //     return -(d*d) ; 
    // };

    /* 5: */
    // const double hip_weight = 0.00009; // added 1x0. better performance on some paths, but less human-like on others. 
    // const double elbow_weight= 0.000000001; 

    // const double energy_weight = 0.0001; 
    // const std::vector<std::string> joint_model_names = joint_model_group->getJointModelNames();
    // const auto cost_fn = [&compute_energy,  &energy_weight]
    //                                             (const geometry_msgs::msg::Pose& /*goal_pose*/,
    //                                             const moveit::core::RobotState& solution_state,
    //                                             moveit::core::JointModelGroup const* jmg,
    //                                             const std::vector<double>& seed_state) {
    //     std::vector<double> proposed_joint_positions;
    //     solution_state.copyJointGroupPositions(jmg, proposed_joint_positions);
    //     double T = compute_energy(seed_state, proposed_joint_positions); 
    //     return T ;
    // };
    
    // /* 6: */
    // const auto frac = moveit::core::CartesianInterpolator::computeCartesianPath(
    //     start_state.get(), joint_model_group, traj, joint_model_group->getLinkModel("can"), targets, true,
    //     max_eef_step, cartesian_precision, callback_fn, opts , cost_fn);

    // RCLCPP_INFO(LOGGER, "\033[32mComputed %f percent of cartesian path.\033[0m", frac.value * 100.0);

    // /* 7: */
    // robot_trajectory::RobotTrajectory rt(start_state->getRobotModel(), PLANNING_GROUP);
    // for (const moveit::core::RobotStatePtr& traj_state : traj)
    //     rt.addSuffixWayPoint(traj_state, 0.0);
    // trajectory_processing::TimeOptimalTrajectoryGeneration time_param;
    // time_param.computeTimeStamps(rt, 1.0);

    // /* 8: */
    // moveit_msgs::msg::RobotTrajectory rt_msg;
    // rt.getRobotTrajectoryMsg(rt_msg);
    // move_group.execute(rt_msg);
    // outputdata = rt_msg; 

    // if(frac.value<1.0 && fully_calc){
    //     RCLCPP_INFO(LOGGER, "Path was not computed fully, attempting to move the last part"); 
    //     move_group.setPoseTarget(waypoints.back());
    //     move_group.clearPathConstraints(); 
    //     moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    //     bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    //     RCLCPP_INFO(LOGGER, "Computation of missing path %s", success? "successful": "\033[31mFAILURE\033[0m");
    //     if(success){
    //         move_group.execute(my_plan); 
    //         outputdata["info"]["endpoint_reachable"]=true;
    //     }
    //     else{
    //         size_t pt = numElements-2;
    //         while(pt>0){
    //             move_group.setPoseTarget(waypoints[pt]);
    //             moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    //             bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    //             if(success){
    //                 int num_points = numElements; 
    //                 int num_points_not_movedto = num_points-pt -1;
    //                 float percentage_reachable = ( (float) pt/(float) num_points) *100.0 ;
    //                 RCLCPP_INFO(LOGGER, "\033[32mTotal movement: %f, with %d/%d Points not reachable \033[0m",percentage_reachable, num_points_not_movedto, num_points);
    //                 move_group.execute(my_plan);
    //                 outputdata["info"]["endpoint_reachable"]=false;
    //                 outputdata["info"]["max_percentage"]=percentage_reachable;
    //                 outputdata["info"]["num_poijoint_states = joint_model_group->nts_moved_to"]=num_points_not_movedto;
    //                 outputdata["info"]["last_pos_movable"]=waypoints[pt]; 
    //                 break;
    //             }
    //             --pt; 
    //         }

    //     }
    // }
    
    // outputdata["final_pose"]= move_group.getCurrentPose().pose;
    // to_json(outputdata["final_jointstates"], move_group); 
    // outputdata["info"]["cartesian_path_completion"]=frac.value;

    // outputdata["info"]["NJS"] = evaluator::calculate_av_NJS(rt_msg, LOGGER); 
    auto input_path_eval = evaluator::endeffector(waypoints, timesteps, LOGGER);
    outputdata["info"]["Endeffector_NJS"] = input_path_eval.get_NJS(); 

    outputfile << std::setw(4) <<outputdata <<std::endl; 


    planning_pipeline.reset();
    psm.reset();
    robotmodelloader.reset();
    robot_state.reset(); 

    rclcpp::shutdown();
    return 0;
}

