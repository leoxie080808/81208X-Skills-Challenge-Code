#include "main.h"
//helper function
void setDrive(int left, int right);



double avgDriveEncoderValue();


//driver controller functions
void setDriveMotors();

void resetDriveEncoder();

//auto stuff
void translate(int units, int voltage);


void turnGyro(float setpoint, float maxPower, float minPower, float kp, float kd, float ki,
					float deadZone, float momentum,int wheelFlag, int timeout);

double turnGyroControlTask();
