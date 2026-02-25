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
#include "ik_processing/srv/data.hpp"
#include "ik_processing/msg/header.hpp"


using Datapoint = ik_processing::msg::Datapoint; 
using Header = ik_processing::msg::Header;
using Data = ik_processing::srv::Data;

class Parser: public rclcpp::Node{
    public:
    Parser(): Node("parser_node")
    {
        RCLCPP_INFO(this->get_logger(), "Starting to parse..."); 
        this->declare_parameter("is_baseline", true);
        _is_baseline = this->get_parameter("is_baseline").as_bool();

        this->declare_parameter("is_agent1", true);
        _is_agent1= this->get_parameter("is_agent1").as_bool();
        
        this->declare_parameter("startpos1", 1);
        _startpos1= static_cast<u_short>(this->get_parameter("startpos1").as_int()); 
        this->declare_parameter("goalpos1", 3);
        _goalpos1 = static_cast<u_short>(this->get_parameter("goalpos1").as_int());
        this->declare_parameter("startpos2", 5);
        _startpos2 = static_cast<u_short>(this->get_parameter("startpos2").as_int());
        this->declare_parameter("goalpos2", 7);
        _goalpos2 = static_cast<u_short>(this->get_parameter("goalpos2").as_int());
        this->declare_parameter("datafile", 1);
        _datafile = this->get_parameter("datafile").as_int();
        this->declare_parameter("exNum", 1);
        _exnum = static_cast<u_short>(this->get_parameter("exNum").as_int());

        if(_is_baseline){
            filepath /= "baselines";
            filepath /= _is_agent1 ? "agent1" : "agent2";
            filepath /= _is_agent1? (std::to_string(_startpos1) + "-" + std::to_string(_goalpos1)) : 
                                    (std::to_string(_startpos2) + "-" + std::to_string(_goalpos2));
            filepath /= (_exnum < 10) ? "00"+ std::to_string(_exnum): "0"+ std::to_string(_exnum); 
            filepath /= std::to_string(_datafile);
            filepath += ".dat";
        }
        else{
            filepath /= "interaction";
            filepath /= std::to_string(_startpos1) +"-"+std::to_string(_goalpos1)+"_"+std::to_string(_startpos2) +"-"+std::to_string(_goalpos2);
            filepath /= (_exnum < 10) ? "00"+ std::to_string(_exnum): "0"+ std::to_string(_exnum);
            filepath /= "1.dat"; 
        }
        
        std::string file_info= create_file_info(); 

        RCLCPP_INFO(this->get_logger(), "\033[35mOpening: %s \033[0m", file_info.c_str()); 

        std::ifstream file{filepath};
        if(file.is_open()){
            RCLCPP_INFO(this->get_logger(), "Successfully opened file");
            data = create_datapoints(file); 
        }
        else{
            RCLCPP_ERROR(this->get_logger(), "Failed to open file with path: %s", filepath.c_str());
        }

        header_ = Header();
        header_.len = data.size(); 
        header_.startpos1 = _startpos1;
        header_.goalpos1 = _goalpos1;
        header_.startpos2 = _startpos2;
        header_.goalpos2 = _goalpos2;
        header_.is_agent1 = _is_agent1; 
        header_.exnum = _exnum; 
        header_.is_baseline= _is_baseline;
        header_.run = _datafile; 

        auto handle_service= [this](const std::shared_ptr<rmw_request_id_t> request_header,
                            const std::shared_ptr<Data::Request> request,
                            const std::shared_ptr<Data::Response> response){
            (void)request_header; 
            (void)request; 
            response ->header = header_;
            response->data = data; 
            RCLCPP_INFO(this->get_logger(), "Packets send, shutting down");
            service_completed_ = true; 
        };
        _service_server = this->create_service<Data>("parse_data", handle_service);
        RCLCPP_INFO(this->get_logger(), "Ready for requests");

    }
    bool is_service_done() const{
        return service_completed_; 
    }
    
    private:
    bool _is_baseline;
    std::filesystem::path filepath = ament_index_cpp::get_package_share_directory("ik_processing") + "/data/dset"; 
    bool _is_agent1;
    u_short _startpos1, _startpos2, _goalpos1, _goalpos2, _exnum, _datafile; 
    std::vector<Datapoint> data; 
    Header header_; 
    std::vector<Datapoint> create_datapoints(std::ifstream& file);
    rclcpp::Service<Data>::SharedPtr _service_server;
    bool service_completed_ =false; 

    std::string create_file_info(){
        using namespace std::string_literals;
        std::string file_info =
            (_is_baseline ? "Baseline "s : "Interaction "s) +
            (_is_agent1 ? "Agent 1 "s : "Agent 2 "s)+
            "Positions: "s +
            ((_is_agent1 && _is_baseline)? std::to_string(_startpos1) + "-"s + std::to_string(_goalpos1) :
                 std::to_string(_startpos2) + "-"s + std::to_string(_goalpos2))+
            (!_is_baseline ? ("_"s + std::to_string(_startpos2) + "-"s + std::to_string(_goalpos2)) : ""s) +
            " Experiment Number: "s + std::to_string(_exnum) +
            (_is_baseline ? (" run: "s + std::to_string(_datafile)) : ""s);
        return file_info; 
    }

};

std::vector<Datapoint> Parser::create_datapoints(std::ifstream& file){
    std::vector<Datapoint> data; 
    std::vector<std::stringstream> vss; 
    std::string line;
    while(std::getline(file, line)){
        vss.emplace_back(line);
    }
    if(_is_baseline && _is_agent1){
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
    }else if(_is_baseline){
        std::array<float,5> f;
        while (vss[0]>>f[0]){
            for(size_t i=1; i<5; ++i){
                vss[i]>>f[i];
            }
            Datapoint d;
            d.time= f[0];
            d.x2=f[1];
            d.y2=f[2];
            d.z2=f[3];
            d.error2=f[4];
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
    auto node= std::make_shared<Parser>();
    while(rclcpp::ok() && !(node->is_service_done())){
        rclcpp::spin_some(node);
    }
    rclcpp::shutdown(); 
    return 0; 
}