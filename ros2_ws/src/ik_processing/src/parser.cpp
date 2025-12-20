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

using Datapoint = ik_processing::msg::Datapoint; 

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

        std::string file_info =
            (_is_baseline ? "Baseline " : "Interaction ") +
            std::string("Positions: ") +
            std::to_string(_startpos1) + "-" + std::to_string(_goalpos1) +
            (!_is_baseline ? ("_" + std::to_string(_startpos2) + "-" + std::to_string(_goalpos2)) : "") +
            " Experiment Number: " + std::to_string(_exnum) +
            (_is_baseline ? (" run: " + std::string(_is_dat1 ? "1" : "2")) : "");
        
        RCLCPP_INFO(this->get_logger(), "Opening: %s", file_info.c_str()); 

        std::ifstream file{filepath};
        if(file.is_open()){
            RCLCPP_INFO(this->get_logger(), "Successfully opened file");
            data = create_datapoints(file, _is_baseline); 
            RCLCPP_INFO(this->get_logger(), "time: %f", data[0].time ); 
        }
        else{
            RCLCPP_INFO(this->get_logger(), "Failed to open file");
        }
        _datapoint_publisher = this->create_publisher<Datapoint> ("datapoint", 100);

        for(Datapoint d: data){
            using namespace std::chrono_literals;
            _datapoint_publisher->publish(d); 
            std::this_thread::sleep_for(10ms);
        }

    }
    
    private:
    bool _is_baseline;
    std::filesystem::path filepath = ament_index_cpp::get_package_share_directory("ik_processing") + "/data/dset"; 
    bool _is_agent1;
    u_short _startpos1, _startpos2, _goalpos1, _goalpos2, _exnum;
    bool _is_dat1; 
    std::vector<Datapoint> data; 
    std::vector<Datapoint> create_datapoints(std::ifstream& file, const bool _is_baseline);
    rclcpp::Publisher<Datapoint>::SharedPtr _datapoint_publisher;

};

std::vector<Datapoint> Parser::create_datapoints(std::ifstream& file, const bool _is_baseline){
        std::vector<Datapoint> data; 
        std::vector<std::stringstream> vss; 
        std::string line;
        while(std::getline(file, line)){
            vss.emplace_back(line);
        }
        if(_is_baseline){
            std::array<float, 5> f;
            while(vss[0]>>f[0]){
                for(size_t i=1; i<5; ++i){
                    vss[i] >> f[i]; 

                }
                Datapoint d;
                d.time=f[0];
                d.x1= f[1];
                d.y1= f[2];
                d.z1= f[3];
                d.error1= f[4];
                data.push_back(d);
            }
            return data; 
        }else{
            std::array<float,9> f;
            while(vss[0]>>f[0]){
                for(size_t i=1; i<9; i++){
                    vss[i] >> f[i];
                }
                Datapoint d;
                d.time=f[0];
                d.x1= f[1];
                d.y1= f[2];
                d.z1= f[3];
                d.error1= f[4];
                d.x2=f[5];
                d.y2=f[6];
                d.z2=f[7];
                d.error2=f[8]; 
                data.push_back(d); 
            } 
        }
        return data; 
    }

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    auto node =(std::make_shared<Parser>());
    rclcpp::shutdown(); 
    return 0; 
}