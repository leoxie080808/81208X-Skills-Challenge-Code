#include "main.h"

void setRollers (int power){
  shooter = power;
  indexer = power;
}




//driver control
void setRollerMotors(){
  int rollerPower = 127*(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2) - controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1));


  setRollers(rollerPower);
}
