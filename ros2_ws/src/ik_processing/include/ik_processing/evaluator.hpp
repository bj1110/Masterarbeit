#pragma once

#include "ik_processing/helpers.hpp"
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace evaluator{

enum LogLevel{verbose, info, error}; 

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
    joint(int arr_pos, const moveit_msgs::msg::RobotTrajectory& rt, const rclcpp::Logger& logger); 
    
    double get_NJS()const {return NJS;};
    std::string get_name()const {return name;};

private: 
    std::vector<double> numeric_derivitive(const std::vector<double>& in, const std::vector<double>& step_durations) const; 
    
};

double calculate_total_NJS(const moveit_msgs::msg::RobotTrajectory& rt, const rclcpp::Logger& logger, LogLevel loglevel=info);
double calculate_av_NJS(const moveit_msgs::msg::RobotTrajectory& rt, const rclcpp::Logger& logger, LogLevel loglevel=info);

class endeffector{
    private:
    std::vector<Eigen::Vector3d> positions_;
    std::vector<Eigen::Vector3d> velocities_;
    std::vector<Eigen::Vector3d> accelerations_;
    std::vector<Eigen::Vector3d> jerks_;
    std::vector<double> timestepsizes_;
    double NJS_; 
    double movementduration_; 
    const rclcpp::Logger& LOGGER_; 

    public: 
    endeffector (const std::vector<geometry_msgs::msg::Pose>& waypoints, const std::vector<double>& timesteps, const rclcpp::Logger& logger);
    double get_NJS()const;

    private:
    std::vector<Eigen::Vector3d> numeric_derivitive(const std::vector<Eigen::Vector3d> in); 
    double calculate_NJS(); 

};

double diff_waypoint_path(const std::vector<geometry_msgs::msg::Pose>& ee_traj, const std::vector<geometry_msgs::msg::Pose>& waypoints, const  std::vector<Datapoint>& data, bool is_agent1);

double calculate_pose_distance(const geometry_msgs::msg::Pose& p1, const geometry_msgs::msg::Pose& p2); 


} //namespace evaluator


namespace builtin_interfaces::msg{

    inline Duration operator-(const Duration& d1, const Duration& d2){
        Duration res;
        res.nanosec= d1.nanosec - d2.nanosec;
        res.sec = d1.sec - d2.sec; 
        return res;
    }


}