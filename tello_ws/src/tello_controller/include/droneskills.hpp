#ifndef DRONE_SKILLS_HPP
#define DRONE_SKILLS_HPP
#include "geometry_msgs/msg/twist.hpp" 
#include "geometry_msgs/msg/pose.hpp"
#include "skills.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"


class DroneSkills : public Skills
{
    public: 

        DroneSkills()
        {
            std::cout << "inicializando drone skills" << std::endl; 
            /* init publishers */
            this->command_topic = create_publisher<std_msgs::msg::String>(
                "/chosen_command", 10
            );
        }

        
        void initialize(); 
        void takeoff();
        void land();
        void up(int x);
        void down(int x);
        void right(int x);
        void left(int x);
        void forward(int x);
        void back(int x);
        void cw(int x);
        void ccw(int x);

        void go(int x, int y, int z, int vel);
        void stop();
        void streamon();
        void streamoff(); 

        void finishChosenSkill();

        void publishCommand(std_msgs::msg::String chosenSkill, std_msgs::msg::Int32 chosenDisplace);

    private:

        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_topic{nullptr}; 

        std_msgs::msg::String command_message{}; 


};


#endif