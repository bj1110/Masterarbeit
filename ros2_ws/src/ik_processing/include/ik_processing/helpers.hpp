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
#include <tf2_eigen/tf2_eigen.hpp>

// for convenience
using json = nlohmann::json;
using Datapoint = ik_processing::msg::Datapoint; 
using Header = ik_processing::msg::Header;
using Data = ik_processing::srv::Data; 


namespace helpers{
    std::ofstream create_file(const rclcpp::Logger& LOGGER, const Header& header );

    void store(json& j, const moveit::planning_interface::MoveGroupInterface& move_group, int datapoint); 

    std::filesystem::path create_datapath(const rclcpp::Logger& LOGGER, const Header& header);

    std::string create_outputname(const Header& header);

    std::string get_startpos(const Header& header);

    inline double to_double(const builtin_interfaces::msg::Duration& d){
        double erg = d.sec + (d.nanosec / 1e9);
        return erg; 
    }

    std::string print_pose(const geometry_msgs::msg::Pose& pose); 

    void dump_gridsearch_result(
        const rclcpp::Logger& LOGGER,
        int turn,
        double angle,
        const Eigen::VectorXd& joint_weights_vector,
        const std::unordered_map<std::string, size_t>& name_to_idx);

    std::vector<geometry_msgs::msg::Pose> robotTrajectory_to_EE_path(const moveit_msgs::msg::RobotTrajectory& rt,
                                                                    moveit::core::RobotState& robot_state,
                                                                    const moveit::core::JointModelGroup* jmg,
                                                                    const std::string& ee_link);
}

inline void to_json(json& j, const moveit::planning_interface::MoveGroupInterface& move_group){
    const std::vector<std::string>&  jointnames = move_group.getJointNames();
    std::vector<double> jointstates = move_group.getCurrentJointValues();
    for(size_t i=0; i< jointnames.size(); ++i){
        j[jointnames[i].c_str()] =  jointstates[i]; 
    }
}


namespace builtin_interfaces::msg{
    inline void to_json(json& j, const Time& t){
        j = {
            {"sec", t.sec},
            {"nanosec", t.nanosec}
        };
    }
    inline void from_json(const json& j, Time& t){
        j.at("sec").get_to(t.sec);
        j.at("nanosec").get_to(t.nanosec);
    }
    inline void to_json(json& j, const Duration& d){
        j = {
            {"sec", d.sec},
            {"nanosec", d.nanosec}
        };
    }
    inline void from_json(const json& j, Duration& d){
        j.at("sec").get_to(d.sec);
        j.at("nanosec").get_to(d.nanosec);
    }
}

namespace moveit_msgs::msg
{
    inline void to_json(json& j, const RobotTrajectory& rt){
        const std::vector<std::string>& jointnames = rt.joint_trajectory.joint_names;
        const std::vector<trajectory_msgs::msg::JointTrajectoryPoint>& jtps = rt.joint_trajectory.points;
        const std_msgs::msg::Header& header = rt.joint_trajectory.header; 
        j["trajectory"]["header"]["stamp"] = header.stamp;
        j["trajectory"]["header"]["frame_id"] = header.frame_id; 
        j["trajectory"]["joint_names"]= jointnames;
        int i=0;
        for(const auto& jtp: jtps ){
            j["trajectory"]["points"][i]["positions"]=jtp.positions;
            j["trajectory"]["points"][i]["velocities"]=jtp.velocities;
            j["trajectory"]["points"][i]["accelerations"]=jtp.accelerations; 
            j["trajectory"]["points"][i]["time_from_start"]=jtp.time_from_start; 
            ++i;      
        }       
    };
    inline void from_json(json& j, RobotTrajectory& rt ){
        if(j.at("trajectory").empty()){
            return;
        }
        std::vector<trajectory_msgs::msg::JointTrajectoryPoint> jtps;
        std::vector<std::string> jointnames; 
        std_msgs::msg::Header header;

        j.at("trajectory").at("joint_names").get_to(jointnames);
        j.at("trajectory").at("header").at("stamp").get_to(header.stamp);
        j.at("trajectory").at("header").at("frame_id").get_to(header.frame_id);

        for(const auto& p: j.at("trajectory").at("points")){
            trajectory_msgs::msg::JointTrajectoryPoint jtp;
            p.at("positions").get_to(jtp.positions);
            p.at("velocities").get_to(jtp.velocities);
            p.at("accelerations").get_to(jtp.accelerations);
            p.at("time_from_start").get_to(jtp.time_from_start);
            jtps.push_back(jtp);
        }
        rt.joint_trajectory.joint_names = jointnames;
        rt.joint_trajectory.points = jtps; 
        rt.joint_trajectory.header = header; 
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

inline void to_json(json& j, const Datapoint& d){
    j={
        {"time", d.time},
        {"x1", d.x1},
        {"y1", d.y1},
        {"z1", d.z1},
        {"error1", d.error1},
        {"x2", d.x2},
        {"y2", d.y2},
        {"z2", d.z2},
        {"error2", d.error2}
    };
}