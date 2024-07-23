#include "skills.hpp"

void Skills::chosenSkillCallback(const std_msgs::msg::Int32 &msg)
{
    this->chosenSkillMessage.data = msg.data; 
}

void Skills::chosenDisplaceCallback(const std_msgs::msg::Int32 &msg)
{
    this->chosenDisplacementMessage.data = msg.data; 
}

std_msgs::msg::Int32 Skills::getChosenSkill()
{
    return this->chosenSkillMessage; 
}

std_msgs::msg::Int32 Skills::getChosenDisplace()
{
    return this->chosenDisplacementMessage;
}

void Skills::executeSkill(int state, int poseDisplacement)
{

    /* verificar o state recebido no callback */
    switch (state)
    {
    case Skills::States::TAKING_OFF:
        this->takeoff(); 
        break;
    
    case Skills::States::GO_LEFT:
        this->left(poseDisplacement);
        break;
    
    case Skills::States::GO_RIGHT:
        this->right(poseDisplacement);
        break;
    
    case Skills::States::GO_FORWARD:
        this->forward(poseDisplacement);
        break; 

    case Skills::States::GO_BACK:
        this->back(poseDisplacement);
        break;

    case Skills::States::GO_UP:
        this->up(poseDisplacement);
        break;

    case Skills::States::GO_DOWN:
        this->down(poseDisplacement);
        break;

    /* não precisa do case 9 executing, pelo menos por enquanto */

    case Skills::States::LANDING: 
        this->land(); 
        break; 

    case Skills::States::LANDED:
        this->takeoff();
        break;

    default:
        break;
    }
}

void Skills::timerCallback()
{

    std::cout << "teste" << std::endl; 
    // executeSkill(getChosenSkill().data, getChosenDisplace().data); 
}
