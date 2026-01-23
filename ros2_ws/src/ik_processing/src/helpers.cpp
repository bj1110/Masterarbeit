#include "ik_processing/helpers.hpp"

#include <fstream>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>

namespace helpers{
    std::ofstream create_file(const rclcpp::Logger& LOGGER ){
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
        std::filesystem::path filepath (dir / "data.json");
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

}

