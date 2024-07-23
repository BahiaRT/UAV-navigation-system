#ifndef TESTE_HPP
#define TESTE_HPP

#include "stdlib.h"
#include "stdio.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "droneskills.hpp"

using namespace std::literals::chrono_literals;

class Teste : public rclcpp::Node
{
    public:

        Teste() : Node("teste")
        {
            this->timer = this->create_wall_timer(
            500ms, std::bind(&Teste::execute, this));

        }
        
        void execute();

    private:

        
        DroneSkills skillsObj{};
        rclcpp::TimerBase::SharedPtr timer{}; 

        bool iniciou{false}; 
        bool landed{true}; 


};

#endif