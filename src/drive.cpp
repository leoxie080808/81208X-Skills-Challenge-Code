#include "main.h"


//driver control functiosn
void setDrive(int left, int right){
  driveLeftBack = left;
  driveLeftFront = left;
  driveRightBack = right;
  driveRightFront = right;
}


void setDriveMotors(){
  int leftJoystick = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
  int rightJoystick = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
  int leftSide = leftJoystick-rightJoystick;
  int rightSide = leftJoystick+rightJoystick;
  setDrive(leftSide, rightSide);
}
