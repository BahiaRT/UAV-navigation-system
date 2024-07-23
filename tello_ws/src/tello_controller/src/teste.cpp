#include "teste.hpp"

void Teste::execute()
{


    if(this->iniciou == false)
    {
        /*
            para escolher a skill que o drone deve executar, basta chamar as funções que estão
            na classe DroneSkills, acessando através do objeto skillsObj. O deslocamento desejado
            deve ser passado como argumento nas skills que necessitam (right, left etc), e deve ser em cm.
        */
        this->skillsObj.initialize(); 
        std::this_thread::sleep_for(2000ms); 
        this->skillsObj.takeoff();
        std::this_thread::sleep_for(8000ms);
        this->skillsObj.land(); 
  
        this->iniciou = true; 
        return;
    }
    

}

int main(int argc, char **argv) 
{
    /* initialize ros node */
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Teste>());
    rclcpp::shutdown();

    return 0;
}