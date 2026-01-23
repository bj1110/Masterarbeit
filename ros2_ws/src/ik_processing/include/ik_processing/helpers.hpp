#pragma once

#include <iostream>
#include <filesystem>
#include <stdexcept>
#include <nlohmann/json.hpp>
#include "ik_processing/msg/datapoint.hpp"
#include "ik_processing/srv/data.hpp"

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// for convenience
using json = nlohmann::json;
using Datapoint = ik_processing::msg::Datapoint; 
using Data = ik_processing::srv::Data; 


namespace helpers{
    std::ofstream create_file(const rclcpp::Logger& LOGGER );

    void store(json& j, const moveit::planning_interface::MoveGroupInterface& move_group, int datapoint); 
}

inline void to_json(json& j, const moveit::planning_interface::MoveGroupInterface& move_group){
    const std::vector<std::string>&  jointnames = move_group.getJointNames();
    std::vector<double> jointstates = move_group.getCurrentJointValues();
    for(size_t i=0; i< jointnames.size(); ++i){
        j["jointstates"][jointnames[i].c_str()] =  jointstates[i]; 
    }

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