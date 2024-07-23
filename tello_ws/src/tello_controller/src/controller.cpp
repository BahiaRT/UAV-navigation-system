#include "controller.hpp"

void Controller::callback_function(const std_msgs::msg::String &msg){
    RCLCPP_INFO(this->get_logger(), "received data: '%s'", msg.data.c_str());
    this->message.data = msg.data;

    this->buffer.data += getMessage().data;
        
    if(this->buffer.data != ""){
        std::cout << "sending message: " << this->buffer.data << std::endl; 
        client.sendMessage(this->buffer.data);//mandar a mensagem do tópico 
        this->buffer.data = "";     
    }
}

std_msgs::msg::String Controller::getMessage(){
    return this->message;
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Controller>());
    rclcpp::shutdown();
    return 0;
}