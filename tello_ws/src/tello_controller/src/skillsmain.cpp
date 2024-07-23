#include "skills.hpp"
#include "droneskills.hpp"

int main(int argc, char *argv[])
{
    std::cout << "SKILLS NODE INITIATED" << std::endl; 

    /* initialize ros node */
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DroneSkills>());
    rclcpp::shutdown();

    return 0;
}