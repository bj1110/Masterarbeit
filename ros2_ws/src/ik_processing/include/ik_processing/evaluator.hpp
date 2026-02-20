#pragma once

#include "ik_processing/helpers.hpp"
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace evaluator{

class joint{
private:
    std::string name;
    double movement_length;
    std::vector<double> jerk;
    double NJS;
    double full_movement_duration; 
    std::vector<double> step_durations; 
    const rclcpp::Logger& LOGGER_;  
    
public: 
    joint(int arr_pos, const moveit_msgs::msg::RobotTrajectory& rt, const rclcpp::Logger& LOGGER); 
    
    double get_NJS()const {return NJS;};
    std::string get_name()const {return name;};
};
double calculate_total_NJS(const moveit_msgs::msg::RobotTrajectory& rt, const rclcpp::Logger& LOGGER);
double calculate_av_NJS(const moveit_msgs::msg::RobotTrajectory& rt, const rclcpp::Logger& LOGGER);



} //namespace evaluator


namespace builtin_interfaces::msg{

    Duration operator-(const Duration& d1, const Duration& d2){
        Duration res;
        res.nanosec= d1.nanosec - d2.nanosec;
        res.sec = d1.sec - d2.sec; 
        return res;
    }


}