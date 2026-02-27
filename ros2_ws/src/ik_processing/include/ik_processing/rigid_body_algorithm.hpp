#pragma once 
#include <memory>
#include <experimental/propagate_const>
#include <rclcpp/rclcpp.hpp> 
		

namespace rigid_body_algorithm{

struct transfer{
    std::vector<double> masses;
    std::vector<std::vector<double>> origin;
    std::vector<std::vector<double>> inertias; 
    size_t num_links;
}; 

class crb{
    private:
    class impl;
    std::experimental::propagate_const<std::unique_ptr<impl>> pimpl;

    public: 
    crb();
    ~crb();
    crb(crb&&);
    crb(const crb&) = delete;
    crb& operator=(const crb&) = delete;
    crb& operator=(crb&&); 

    void crba();

    bool update();

    //TODO: Getter 



};



} //namespace rigid_body_algorithm