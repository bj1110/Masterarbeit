#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <rclcpp/rclcpp.hpp>
#include <variant>
#include <filesystem>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "ik_processing/msg/datapoint.hpp"

struct Baseline_point{
    float time;
    float x, y, z, error; 
};
struct Interaction_point{
    float time;
    float x1, y1, z1, error1, x2, y2, z2, error2; 
};


class Parser: public rclcpp::Node{
    public:
    Parser(): Node("parser_node")
    {
        this->declare_parameter("is_baseline", true);
        _is_baseline = this->get_parameter("is_baseline").as_bool();

        this->declare_parameter("is_agent1", true);
        _is_agent1= this->get_parameter("is_agent1").as_bool();
        
        this->declare_parameter("startpos1", 1);
        _startpos1= static_cast<u_short>(this->get_parameter("startpos1").as_int()); 
        this->declare_parameter("goalpos1", 3);
        _goalpos1 = static_cast<u_short>(this->get_parameter("goalpos1").as_int());
        this->declare_parameter("startpos2", 1);
        _startpos2 = static_cast<u_short>(this->get_parameter("startpos2").as_int());
        this->declare_parameter("goalpos2", 1);
        _goalpos2 = static_cast<u_short>(this->get_parameter("goalpos2").as_int());
        this->declare_parameter("is_dat1", true);
        _is_dat1 = this->get_parameter("is_dat1").as_bool();
        this->declare_parameter("exNum", 1);
        _exnum = static_cast<u_short>(this->get_parameter("exNum").as_int());

        if(_is_baseline){
            filepath /= "baselines";
            filepath /= _is_agent1 ? "agent1" : "agent2";
            filepath /= std::to_string(_startpos1) + "-" + std::to_string(_goalpos1);
            filepath /= (_exnum < 10) ? "00"+ std::to_string(_exnum): "0"+ std::to_string(_exnum); 
            filepath /= _is_dat1 ? "1" :"2";
            filepath += ".dat";
        }
        else{
            filepath /= "interaction";
            filepath /= std::to_string(_startpos1) +"-"+std::to_string(_goalpos1)+"_"+std::to_string(_startpos2) +"-"+std::to_string(_goalpos2);
            filepath /= (_exnum < 10) ? "00"+ std::to_string(_exnum): "0"+ std::to_string(_exnum);
            filepath /= "1.dat"; 
        }
        
        RCLCPP_INFO(this->get_logger(), "Default baseline value: %s", _is_baseline ? "true": "false"); 
        RCLCPP_INFO(this->get_logger(), "Filepath: %s", filepath.c_str());

        std::ifstream file{filepath};
        if(file.is_open()){
            RCLCPP_INFO(this->get_logger(), "Successfully opened file");
            data = create_baseline(file); 
            std::visit([this](const auto& vec){
                const auto& first = vec.at(2);
                RCLCPP_INFO(this->get_logger(), "time: %f", first.time); 
            }, data); 
            
        }
        else{
            RCLCPP_INFO(this->get_logger(), "Failed to open file");
        }
    }
    
    private:
    bool _is_baseline;
    std::filesystem::path filepath = ament_index_cpp::get_package_share_directory("ik_processing") + "/data/dset"; 
    bool _is_agent1;
    u_short _startpos1, _startpos2, _goalpos1, _goalpos2, _exnum;
    bool _is_dat1; 
    std::variant<std::vector<Baseline_point>, std::vector<Interaction_point>> data; 
    
    std::variant<std::vector<Baseline_point>, std::vector<Interaction_point>> create_baseline(std::ifstream& file);

};

std::variant<std::vector<Baseline_point>, std::vector<Interaction_point>> Parser::create_baseline(std::ifstream& file){
        std::variant<std::vector<Baseline_point>, std::vector<Interaction_point>> result; 
        std::vector<std::stringstream> vss; 
        std::string line;
        while(std::getline(file, line)){
            vss.emplace_back(line);
        }
        if(_is_baseline){
            std::array<float, 5> f;
            std::vector<Baseline_point> points;
            while(vss[0]>>f[0]){
                for(size_t i=1; i<5; ++i){
                    vss[i] >> f[i]; 

                }
                Baseline_point bp {f[0], f[1], f[2], f[3], f[4]};
                points.emplace_back(bp);
            }
            result= points; 
        }else{
            std::array<float,9> f;
            std::vector<Interaction_point> points; 
            while(vss[0]>>f[0]){
                for(size_t i=1; i<9; i++){
                    vss[i] >> f[i];
                }
                Interaction_point ip {f[0], f[1], f[2], f[3], f[4], f[5], f[6], f[7], f[8]};
                points.emplace_back(ip);
            }
            result= points; 
        }
        return result; 
    }

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    auto node =(std::make_shared<Parser>());
    rclcpp::shutdown(); 
    return 0; 
}