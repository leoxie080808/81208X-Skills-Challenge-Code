#include "main.h"
#include "drive.hpp"
pros::Imu Gyro(3);



float targetAngle = 0;
int turnTaskStartFlag = 0;
float turnMaxPower = 127;
float turnMinPower = 30;
float turnKp = 2.0;
float turnKd = 0.0;
float turnKi = 0.0;
float turnDeadZone = 10;  // one degree
float turnMomentum = 0;			//
int turnWheelFlag = 0;  // 0 - 2 wheels turn, 1 - left wheel turn 2 - right wheel turn
float robotHeading = 0;

int driveStraightSpeed = 100;
float driveDistance = 0;
float driveSetDistance = 0;
int goStraightStartFlag = 0;
int goStraightAngle = 0;
int startLeftEncoder = 0;
int startRightEncoder = 0;
//go straight value
float goKp = 1.5;
float goKd = 0;
float goKi = 0;
float setMinSpeed = 30;


#define TURN_TASK_PERIOD			10	//10ms control period
#define GO_STRAIGHT_PERIOD		    20   //20ms control period
#define GYRO_RANGE 						7200
#define GYRO_SCALE						138  //need calibrate for each gyro
#define TURN_TASK_DEADZONE	  10	 //1 degree
#define TURN_POWER						60
#define TURN_TASK_MOMENTUM		30		//3 degree
#define WHEEL_RADIUS 					(0.106/ 2)    //(0.102/ 2)		//cm			//need test
#define CIRCLE_COUNT					 360.0  //(360.0 * 60 / 12)   // (60 / 12)

#define MAX_MOVE_SPEED		100.0
#define MIN_MOVE_SPEED		5		// need test
#define GO_DEACC_RATE			2
#define GO_ACC_RATE				10
#define STOP_DISTANCE			0.8
#define GO_SLOWDOWN_RATIO		((float)(MAX_MOVE_SPEED - MIN_MOVE_SPEED) / STOP_DISTANCE)		// which means the max speed to stop min speed in 0.5m 			// 127 - MIN_MOVE_SPEED

# define M_PI           3.14159265358979323846  /* pi */
# define TICKS_PER_REV 900 //18:1 ratio
# define WHEEL_DIAMETER 4.025 //inch
# define DIST_BETWEEN_WHEEL 16 //inch
# define DIST_PER_TICK (2*WHEEL_DIAMETER*M_PI)/TICKS_PER_REV //Inch per tick in encoder

int dist2Encoder(int inch, int distPerTick){
    return inch/distPerTick;
}

/*
0 - On spot turn
1 - Left turn
2 - right turn
This will take the relative angle
*/




//driver control functiosn
void setDrive(int left, int right){
  driveLeftBack = left;
  driveLeftFront = left;
  driveRightBack = right;
  driveRightFront = right;
}


void resetDriveEncoder(){
  driveLeftBack.tare_position();
  driveLeftFront.tare_position();
  driveRightBack.tare_position();
  driveRightFront.tare_position();
}


double avgDriveEncoderValue(){
  return (fabs(driveLeftFront.get_position()) + fabs(driveLeftBack.get_position()) + fabs(driveRightFront.get_position())
  + fabs(driveRightBack.get_position()))/ 4;
}



void setDriveMotors(){
  int leftJoystick = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
  int rightJoystick = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
  int leftSide = leftJoystick+(rightJoystick*1.1);
  int rightSide = leftJoystick-(rightJoystick*1.1);
  setDrive(leftSide, rightSide);
}

template <typename T> int sgn(T val){
  return (T(0) < val) - (val < T(0));
}


//auto stuff
void translate(int units, int voltage){
  int direction = abs(units) / units;
  resetDriveEncoder();


  //reset motor encoders
  while(avgDriveEncoderValue() < abs(units)){
    setDrive(voltage * direction, voltage * direction);
    pros::delay(10);
  }

  setDrive(-10 * direction, -10*direction);
  pros::delay(50);

  setDrive(0,0);
}

double turnGyroControlTask() {
    double ctrlValue = 0;
    double error = 0;
	while(turnTaskStartFlag == 1)
	{
		if (fabs(Gyro.get_heading() - targetAngle) < turnDeadZone)
		{
			setDrive(0,0);
			turnTaskStartFlag = 0;
		}
		else{
            error = targetAngle - Gyro.get_heading();
            if (fabs(error) < 10)
            {
                ctrlValue = sgn(error) * turnMinPower;

            }
            else
            {
                ctrlValue = turnKp * error;
                //2 wheels turn here
                ctrlValue = (fabs(ctrlValue) > turnMaxPower?sgn(ctrlValue) * turnMaxPower:ctrlValue);
                ctrlValue = (fabs(ctrlValue) < turnMinPower? sgn(ctrlValue) * turnMinPower:ctrlValue);
            }
                if (turnWheelFlag == 0)
                {
                    // 2 wheel turns
                    setDrive(ctrlValue,  -ctrlValue);
                }
                else if(turnWheelFlag == 1)
                {
                    // left wheel turns
                    setDrive(ctrlValue, 0);
                }
                else if(turnWheelFlag == 2)
                {
                    // right wheel turns
                    setDrive(0, -ctrlValue);
                }

		}
		pros::delay(10);

	}

    return 0;
}






void turnGyro(float setpoint, float maxPower, float minPower, float kp, float kd, float ki,
					float deadZone, float momentum, int wheelFlag)
{
	int delayCnt = 0;
	if (setpoint >= Gyro.get_heading()){
		// turn right?
		targetAngle = setpoint - momentum;

	}else
	{
		//turn left
		targetAngle = setpoint + momentum;
	}

	turnMaxPower = maxPower;
	turnMinPower = minPower;
	turnKp = kp;
	turnKd = kd;
	turnKi = ki;
	turnDeadZone = deadZone;
	turnMomentum = momentum;
	turnWheelFlag = wheelFlag;

	turnTaskStartFlag = 1;
  pros::Task turnTask(turnGyroControlTask, "turning task");
//	pros::delay(10);
  while (turnTaskStartFlag ==1){
    pros::delay(20);
  }

}
