#include "main.h"


//helper funcion
void setIntake (int power){
  intakeLeft = power;
  intakeRight = power;
}




//driver control
void setIntakeMotors(){
  int intakePower = 127*(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2) - controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1));
  //if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
  //  intakePower = -127;
  //else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1))
//    intakePower = 127;
  setIntake(intakePower);
}
