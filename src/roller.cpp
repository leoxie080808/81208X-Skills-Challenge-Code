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



void score(){
  shooter.move_relative(800, 127);
  indexer.move_relative(800, 127);
  //setRollers(127);
}
