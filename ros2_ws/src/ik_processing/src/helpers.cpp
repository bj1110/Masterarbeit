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
    std::filesystem::path dir = std::filesystem::temp_directory_path()/"simdata";
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
    name += "agent" ; 
    name += header.is_agent1 ? "1_":"2_";
    name += std::to_string(header.startpos1) + "-" + std::to_string(header.goalpos1);
    if(!header.is_baseline){
        name += "_" + std::to_string(header.startpos2) + "-" + std::to_string(header.goalpos2);
    }
    name+="#"+std::to_string(header.run); 
    name += ".json"; 
    return name; 
}

std::string get_startpos(const Header& header){
    if(header.is_agent1){
        u_short s = header.startpos1;
        std::string pos_name = "Pos"+std::to_string(s)+"_right";
        return pos_name;
    }
    else{
        u_short s = header.startpos2;
        if(header.is_baseline){
            s=header.startpos1;
        }
        if(s==5){
            s=1;
        } else if(s==7){
            s=3;
        }
        std::string pos_name = "Pos"+std::to_string(s)+"_right";
        return pos_name;
    }
    return {}; 
}

std::string print_pose(const geometry_msgs::msg::Pose& pose){
    std::ostringstream oss;
    oss << "translational: " << pose.position.x <<", " <<pose.position.y << ", "<< pose.position.z <<"\n";
    oss << "rotational: x="<<pose.orientation.x << ", y="<<pose.orientation.y<< ", z= "<<pose.orientation.z << ", w="<<pose.orientation.w;
    return oss.str(); 
}

void dump_gridsearch_result(
    const rclcpp::Logger& LOGGER,
    int turn,
    double angle,
    const Eigen::VectorXd& joint_weights_vector,
    const std::unordered_map<std::string, size_t>& name_to_idx) 
{ 
    std::filesystem::path dir = std::filesystem::temp_directory_path()/"simdata";
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);
    if (ec) {
        RCLCPP_ERROR(LOGGER, "Failed to create %s: %s", dir.c_str(), ec.message().c_str());
        return;
    }

    std::filesystem::path filepath = dir / "grid_search_data_11.jsonl";

    std::ofstream file(filepath, std::ios::app);
    if (!file.is_open()) {
        RCLCPP_ERROR(LOGGER, "Failed to open: %s", filepath.c_str());
        return;
    }

    json dump_data = {
        {"turn", turn},
        {"angle", angle},
        {"glenohumeral_pitch_joint", joint_weights_vector[(name_to_idx.at("glenohumeral_pitch_joint"))]},
        {"sternoclavicular_pitch_joint", joint_weights_vector[(name_to_idx.at("sternoclavicular_pitch_joint"))]}
    };

    file << dump_data.dump() << "\n";

    RCLCPP_INFO(LOGGER, "Appended grid search result");
}

std::vector<geometry_msgs::msg::Pose> robotTrajectory_to_EE_path(const moveit_msgs::msg::RobotTrajectory& rt,
                                                                    moveit::core::RobotState& robot_state,
                                                                    const moveit::core::JointModelGroup* jmg,
                                                                    const std::string& ee_link)
{
    std::vector<geometry_msgs::msg::Pose> ee_path;

    for (const auto& p : rt.joint_trajectory.points)
    {
        robot_state.setJointGroupPositions(jmg, p.positions);
        robot_state.update();

        const Eigen::Isometry3d& tf =
            robot_state.getGlobalLinkTransform(ee_link);

        geometry_msgs::msg::Pose pose =
            tf2::toMsg(tf);

        ee_path.push_back(pose);
    }

    return ee_path;
}

} //namespace helpers 

