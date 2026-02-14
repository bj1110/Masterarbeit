#include "ik_processing/helpers.hpp"

#include <fstream>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>

namespace helpers{

std::ofstream create_file(const rclcpp::Logger& LOGGER, const Header& header ){
    std::filesystem::path filepath = create_datapath(LOGGER, header); 
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

void store(json& j, const moveit::planning_interface::MoveGroupInterface& move_group, int datapoint){
    
    std::string name= (datapoint <10)? "Datapoint" + std::to_string(0) +std::to_string(datapoint):"Datapoint" + std::to_string(datapoint) ;
    j[name]["Pose"]= move_group.getCurrentPose().pose;
    j[name]["Jointvalues"]=move_group.getCurrentJointValues(); 
}

std::filesystem::path create_datapath(const rclcpp::Logger& LOGGER, const Header& header){
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
    std::string filename = create_outputname(header);
    std::filesystem::path filepath (dir / filename);
    return filepath; 
}

std::string create_outputname(const Header& header){
    std::string name = header.is_baseline ? "base" : "interaction";
    name += std::to_string(header.exnum);
    name += "agent" + std::to_string(header.agent)+ "_";
    name += std::to_string(header.startpos1) + "-" + std::to_string(header.goalpos1);
    if(!header.is_baseline){
        name += "_" + std::to_string(header.startpos2) + "-" + std::to_string(header.goalpos2);
    }
    name += ".json"; 
    return name; 
}

std::string get_startpos(const Header& header){
    if(header.agent==1){
        u_short s = header.startpos1;
        std::string pos_name = "Pos"+std::to_string(s)+"_right";
        return pos_name;
    }
    if(header.agent==2){
        u_short s = header.startpos1;
        std::string pos_name = "Pos"+std::to_string(s)+"_right";
        return pos_name;
    }
    return {}; 
}


} //namespace helpers 

