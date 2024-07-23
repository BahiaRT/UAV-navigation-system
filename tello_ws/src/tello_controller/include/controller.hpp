#ifndef CONTROLLER_HPP 
#define CONTROLLER_HPP
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <sstream>
#include "../network/udpsocket.hpp"

using namespace std::chrono_literals;

class Controller : public rclcpp::Node{
    public:
        Controller() : Node("controlador")
        {
            std::cout << "CONTROLLER INICIOU" << std::endl; 
            this->client.setupAddress("192.168.10.1", "8889");
            this->command_sub = this->create_subscription<std_msgs::msg::String>(
            "/chosen_command", 10, std::bind(&Controller::callback_function, this, std::placeholders::_1));
            this->timer = this->create_wall_timer(500ms, std::bind(&Controller::getMessage, this));
        }

        std_msgs::msg::String getMessage();

        void callback_function(const std_msgs::msg::String &msg);


    private:

        std_msgs::msg::String message{};
        std_msgs::msg::String buffer{};
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub;
        bahiart::NetworkManager::UdpSocket client{};
        rclcpp::TimerBase::SharedPtr timer{};
};

#endif