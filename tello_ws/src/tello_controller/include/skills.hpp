#ifndef SKILLS_HPP
#define SKILLS_HPP
#include <stdio.h>
#include <stdlib.h>
#include <chrono>
#include <string>
#include <functional>
#include <memory>
#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp" 
#include "geometry_msgs/msg/pose.hpp"
#include "std_msgs/msg/int32.hpp"

class Skills : public rclcpp::Node
{
    public:

        /* constructor */
        Skills() : Node("skills")
        {
            /* init subscribers */ 
            this->chosenSkillTopic = this->create_subscription<std_msgs::msg::Int32>(
                "/manager_chosen_skill", 10, 
                std::bind(&Skills::chosenSkillCallback, this, std::placeholders::_1)
            );

            this->chosenDisplaceTopic = this->create_subscription<std_msgs::msg::Int32>(
             "/manager_chosen_displacement", 10,
             std::bind(&Skills::chosenDisplaceCallback, this, std::placeholders::_1)    
            );

         
        }; 


        virtual void initialize()=0; 
        virtual void takeoff()=0;
        virtual void land()=0;
        virtual void up(int x)=0;
        virtual void down(int x)=0;
        virtual void right(int x)=0;
        virtual void left(int x)=0;
        virtual void forward(int x)=0;
        virtual void back(int x)=0;
        virtual void cw(int x)=0;
        virtual void ccw(int x)=0;
        virtual void finishChosenSkill()=0;

        virtual void executeSkill(int state, int poseDisplacement); 

        void chosenSkillCallback(const std_msgs::msg::Int32 &msg);

        void chosenDisplaceCallback(const std_msgs::msg::Int32 &msg); 

        std_msgs::msg::Int32 getChosenSkill();

        std_msgs::msg::Int32 getChosenDisplace(); 

        void timerCallback(); 

        enum States
        {
                NO_STATE = -1,
                NOT_INITIALIZED=1,
                TAKING_OFF, //2
                GO_LEFT, //3
                GO_RIGHT, //4
                GO_FORWARD, //5
                GO_BACK, //6
                GO_UP, //7
                GO_DOWN, //8
                EXECUTING, //9 
                LANDING, //10
                LANDED    //11
        };

    protected: 

        /* subscreve no estado da skill escolhida */
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr chosenSkillTopic{nullptr};

        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr chosenDisplaceTopic{nullptr}; 

        std_msgs::msg::Int32 chosenSkillMessage{}; 

        std_msgs::msg::Int32 chosenDisplacementMessage{}; 
         

       
};

#endif