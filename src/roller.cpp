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
  shooter.move_relative(1200, 600);
  indexer.move_relative(2100, 600);
  //setRollers(127);
}
