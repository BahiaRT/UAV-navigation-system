#include "droneskills.hpp"

void DroneSkills::initialize()
{
   /* send "command" to drone to initialize */
   std::cout << "publishing message " << std::endl; 
   std_msgs::msg::String message{};
   message.data = "command";
   this->command_topic->publish(message); 
}

void DroneSkills::takeoff()
{
    std_msgs::msg::String message{};
    message.data = "takeoff";
    this->command_topic->publish(message);  
}

void DroneSkills::land()
{
    std_msgs::msg::String message{};
    message.data = "land";
    this->command_topic->publish(message); 
}

void DroneSkills::up(int x)
{
    std_msgs::msg::String message{};
    message.data = "up "+std::to_string(x); 
    this->command_topic->publish(message); 

}
void DroneSkills::down(int x)
{
    std_msgs::msg::String message{};
    message.data = "down "+std::to_string(x); 
    this->command_topic->publish(message); 
}

void DroneSkills::right(int x)
{
    std_msgs::msg::String message{};
    message.data = "right "+std::to_string(x); 
    this->command_topic->publish(message); 
}

void DroneSkills::left(int x)
{
    std_msgs::msg::String message{};
    message.data = "left "+std::to_string(x); 
    this->command_topic->publish(message); 

}

void DroneSkills::forward(int x)
{
    std_msgs::msg::String message{};
    message.data = "forward "+std::to_string(x); 
    this->command_topic->publish(message); 
}

void DroneSkills::back(int x)
{
    std_msgs::msg::String message{};
    message.data = "back "+std::to_string(x); 
    this->command_topic->publish(message); 
}


void DroneSkills::cw(int x)
{
    std_msgs::msg::String message{};
    message.data = "cw "+std::to_string(x); 
    this->command_topic->publish(message); 
}

void DroneSkills::ccw(int x)
{
    std_msgs::msg::String message{};
    message.data = "ccw "+std::to_string(x); 
    this->command_topic->publish(message); 
}

void DroneSkills::go(int x, int y, int z, int vel)
{
    std_msgs::msg::String message{};
    message.data = "go "+
    std::to_string(x)+ " " +
    std::to_string(y)+ " " +
    std::to_string(z)+ " " +
    std::to_string(vel);
    this->command_topic->publish(message);  
}

void DroneSkills::stop()
{
    std_msgs::msg::String message{};
    message.data = "stop";
    this->command_topic->publish(message);
}

void DroneSkills::streamon()
{
    std_msgs::msg::String message{};
    message.data = "streamon";
    this->command_topic->publish(message); 
}

void DroneSkills::streamoff()
{
    std_msgs::msg::String message{};
    message.data = "streamoff";
    this->command_topic->publish(message); 
}

void DroneSkills::finishChosenSkill()
{
    
}

void DroneSkills::publishCommand(std_msgs::msg::String chosenSkill, std_msgs::msg::Int32 chosenDisplace)
{
    /* publicar a skill escolhida no tópico para o controller que irá enviar o comando para o drone */


}