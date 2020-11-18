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


bool turnEndStop = true;

//int turnTaskStartFlag = 0;
float slowTurnAngle = 10;




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




double currentHeading = 0;

/*
0 - On spot turn
1 - Left turn
2 - right turn
This will take the relative angle
*/





double get_heading() {
	double heading = Gyro.get_heading();
	if (heading > 180)
		return heading - 360;
	else
		return heading;
}







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

double turnGyroControlTask()
{
  	float ctrlValue = 0;
//  #ifdef INERTIAL_SENSOR
    currentHeading = get_heading();//Gyro.get_heading();
//  #else
	// currentHeading =  myGyro.controllerGet();
//  #endif
  float error = targetAngle - currentHeading;
	float slowRatio = 0;
	if (fabs(error) > slowTurnAngle)
	{
		slowRatio = (turnMaxPower - turnMinPower) / (fabs(error) - slowTurnAngle);
	}

	while(turnTaskStartFlag == 1)
	{
  //  #ifdef INERTIAL_SENSOR
      currentHeading = get_heading();
  //  #else
  	// currentHeading =  myGyro.controllerGet();
  //  #endif
		if (	fabs(currentHeading - targetAngle) < turnDeadZone)
		{
			// finish turn

			if (turnEndStop){
				setDrive(0,0);
			}
			turnTaskStartFlag = 0;
		}
		else{
			error = targetAngle - currentHeading;

			if (fabs(error) <= slowTurnAngle)
			{
				ctrlValue = sgn(error) * turnMinPower;


			}
			else
			{
        /*
					ctrlValue = turnKp * error;
					if (ctrlValue > turnMaxPower){
						ctrlValue = turnMaxPower;
					}
					else{
						ctrlValue = turnKp * error;
					}*/
				ctrlValue = sgn(error) * (turnMinPower + ( fabs(error) - slowTurnAngle) * slowRatio);
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

    pros::delay(TURN_TASK_PERIOD);

	}
}










void turnGyro(float setpoint, float maxPower, float minPower, float kp, float kd, float ki,
					float deadZone, float momentum, int wheelFlag, int timeOut){
	int delayCnt = 0;
	if (setpoint >= get_heading()){
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
  while (turnTaskStartFlag ==1){
    delayCnt++;
    if (delayCnt > timeOut / TURN_TASK_PERIOD)
    {

      turnTaskStartFlag = 0;
      break;
    }
    pros::delay(10);
  }
//	pros::delay(10);
  setDrive(0,0);
  pros::delay(100);
//  return 0;
}





//here is oging forward stuff
#define ACC_MIN_SPEED			25
#define GO_ACCEL_DIS			0.1
#define GO_ACCEL_RATIO		((float)(MAX_MOVE_SPEED - ACC_MIN_SPEED) / GO_ACCEL_DIS)		// which means the max speed to stop min speed in 0.5m


#define WHEEL_PERIMETER		0.319 	//4"
#define CIRCLE_COUNT			360  // 900,300
#define MAX_MOVE_SPEED		200
#define MIN_MOVE_SPEED		 	10	// need test


#define STOP_DISTANCE			0.15
#define GO_SLOWDOWN_RATIO		((float)(MAX_MOVE_SPEED - MIN_MOVE_SPEED) / STOP_DISTANCE)		// which means the max speed to stop min speed in 0.5m 			//
//#define GO_MOMENTUM				0.04
#define SLOW_DRIVE_DIS		0.0			//10cm

bool goEndStop = true;


int startLeftEncoderF = 0;
int startLeftEncoderR = 0;
int startRightEncoderF = 0;
int startRightEncoderR = 0;

void goStraightGyroControlTask()
{


	double deltaLeft = 0;
	double deltaRight = 0;

	float driveDistance = 0;
	float goSpeed = driveStraightSpeed;

	float leftCmd = 0;
	float rightCmd = 0;

  float GO_SLOWDOWN_DIS = (float)(abs(driveStraightSpeed) - MIN_MOVE_SPEED) / GO_SLOWDOWN_RATIO;
  float go_accel_dis = (float)(abs(driveStraightSpeed) - ACC_MIN_SPEED) / GO_ACCEL_RATIO;


	float remainDis = 0 ;
	float dynamicKp = goKp;
	float ctrlValue = 0;
  int curLeft = 0;
  int curRight = 0;

	while(goStraightStartFlag == 1){
			remainDis = fabs(driveSetDistance - driveDistance);
			if(driveDistance > (driveSetDistance - GO_SLOWDOWN_DIS))
			{

			 if (remainDis <= SLOW_DRIVE_DIS)
			 {
			   goSpeed = sgn(goSpeed) * MIN_MOVE_SPEED;
			 }
			 else
			 {

			 		goSpeed = sgn(driveStraightSpeed) * MIN_MOVE_SPEED + sgn(driveStraightSpeed) * (remainDis - SLOW_DRIVE_DIS)  * GO_SLOWDOWN_RATIO;

				 if (fabs(goSpeed) < MIN_MOVE_SPEED)
				 {
				   		goSpeed = sgn(goSpeed) * MIN_MOVE_SPEED;
				 }
				}

			}
			else
			{
				if (driveDistance < GO_ACCEL_DIS){
					goSpeed = sgn(driveStraightSpeed) *  ACC_MIN_SPEED + sgn(driveStraightSpeed) * driveDistance  * GO_ACCEL_RATIO;
					if (fabs(goSpeed) > abs(driveStraightSpeed)) goSpeed = driveStraightSpeed;
					//writeDebugStreamLine("D = %f, v = %f", driveDistance, goSpeed);
				}
				else{

					goSpeed = driveStraightSpeed;
				}
			}
		dynamicKp = (0.5 + 0.5 * fabs(goSpeed / MAX_MOVE_SPEED)) * goKp;
  //  #ifdef INERTIAL_SENSOR
    //  currentHeading = sys->get_heading();
  //  #else
  	 currentHeading =  get_heading();//Gyro.get_heading();
  //  #endif
		ctrlValue = dynamicKp * (goStraightAngle - currentHeading);

		leftCmd = goSpeed + ctrlValue ;
		rightCmd = goSpeed - ctrlValue ;


		deltaLeft = (fabs( driveLeftFront.get_position()+driveLeftBack.get_position() - startLeftEncoderF - startLeftEncoderR)) / 2.0 ;
		deltaRight = (fabs( driveRightFront.get_position()+driveRightBack.get_position() - startRightEncoderF - startRightEncoderR)) / 2.0;

		driveDistance =  ((float)(deltaLeft + deltaRight) /2.0 ) * WHEEL_PERIMETER / CIRCLE_COUNT;


		if(driveDistance < driveSetDistance){
			//keep drive
			setDrive(leftCmd, rightCmd);
		}else{
			//stop motor and the task
			if (goEndStop){
				setDrive(0,0);
			}
			goStraightStartFlag = 0;

		}
    pros::delay(GO_STRAIGHT_PERIOD);

	}
}


void driveStraight( float distance, float setAngle, float power, float kp, float timeout,int timeOut, float momentum ,bool endStop){

	driveSetDistance = distance;

	if (distance >= momentum)
		driveSetDistance = distance - momentum;			// 1m
	else
		driveSetDistance = distance;


	driveStraightSpeed = power;		//power
	startLeftEncoderF = driveLeftFront.get_position();
  startLeftEncoderR = driveLeftBack.get_position();//leftRearMotor->getEncoder()->controllerGet();

	startRightEncoderF = driveRightFront.get_position();//rightFwdMotor->getEncoder()->controllerGet();
  startRightEncoderR = driveRightBack.get_position();//rightRearMotor->getEncoder()->controllerGet();
	goStraightAngle = setAngle;
	goStraightStartFlag = 1;
	goKp = kp;
	goEndStop = endStop;
	int delayCnt = 0;

  pros::Task go_straigt_task(goStraightGyroControlTask, "go straight task");

  int time = timeOut * 1000 / GO_STRAIGHT_PERIOD;
	while(goStraightStartFlag == 1){
		delayCnt ++;
		if (delayCnt > time){
			setDrive(0,0);
			goStraightStartFlag = 0;


		}
    pros::delay(GO_STRAIGHT_PERIOD);

	}


}
