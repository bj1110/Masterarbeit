#pragma once

#include <iostream>
#include <filesystem>
#include <stdexcept>
#include <nlohmann/json.hpp>
#include "ik_processing/msg/datapoint.hpp"
#include "ik_processing/msg/header.hpp"
#include "ik_processing/srv/data.hpp"

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// for convenience
using json = nlohmann::json;
using Datapoint = ik_processing::msg::Datapoint; 
using Header = ik_processing::msg::Header;
using Data = ik_processing::srv::Data; 


namespace helpers{
    std::ofstream create_file(const rclcpp::Logger& LOGGER, const Header& header );

    void store(json& j, const moveit::planning_interface::MoveGroupInterface& move_group, int datapoint); 

    std::filesystem::path create_datapath(const rclcpp::Logger& LOGGER);

    std::string create_outputname(const Header& header);
}

inline void to_json(json& j, const moveit::planning_interface::MoveGroupInterface& move_group){
    const std::vector<std::string>&  jointnames = move_group.getJointNames();
    std::vector<double> jointstates = move_group.getCurrentJointValues();
    for(size_t i=0; i< jointnames.size(); ++i){
        j["jointstates"][jointnames[i].c_str()] =  jointstates[i]; 
    }

}

namespace moveit_msgs::msg
{
    inline void to_json(json& j, const RobotTrajectory& rt){
        std::vector<std::string> jointnames = rt.joint_trajectory.joint_names;
        std::vector<trajectory_msgs::msg::JointTrajectoryPoint> jtps = rt.joint_trajectory.points;
        size_t numjoints = jointnames.size(); 
        for (size_t i=0; i<jtps.size(); ++i){
            for(size_t k=0; k< numjoints; ++k){
                j[i][jointnames[k].c_str()] ["position"]= jtps[i].positions[k]; 
                j[i][jointnames[k].c_str()] ["velocity"]= jtps[i].velocities[k]; 
                j[i][jointnames[k].c_str()] ["acceleration"]= jtps[i].accelerations[k]; 
            }
        }
    };
}

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