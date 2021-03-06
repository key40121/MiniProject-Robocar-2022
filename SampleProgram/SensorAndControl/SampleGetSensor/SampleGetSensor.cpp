//============================================================================
// Name        : SampleGetSensor.cpp
// Author      :
// Version     :
// Copyright   : Your copyright notice
// Description : Aquire some data from every sensors
//============================================================================

#include <iostream>
#include <termios.h>
#include <fcntl.h>
#include <strings.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include "RcControl.h"

using namespace zmp::zrc;

int main(){
	RcControl _RcControl;
	DRIVE_VALUE drive;
	SENSOR_VALUE sensor;
	OBSTACLE_VALUE obstacle;
	POWER_VALUE power;
	THERMO_VALUE thermo;
	int frame;

	_RcControl.init();
	_RcControl.Start();
	_RcControl.SetReportFlagReq(0x0f);
	_RcControl.SetMotorEnableReq(0);
	_RcControl.SetDriveSpeed(0);

	memset(&power, 0, sizeof(POWER_VALUE));
	sleep(2);
	_RcControl.GetSensorInfoReq(&sensor);
	printf("before gyro=%3.2f\n", sensor.gyro);
	_RcControl.GetPowerInfoReq(&power);
	printf("before current=%3.2f\n", power.motor_current);
	_RcControl.SetGyroOffset(sensor.gyro);
	_RcControl.SetMotorCurrentOffset(power.motor_current);
	_RcControl.SetMotorEnableReq(1);
	
	frame = 0;

	while(1){
		frame++;
		usleep(1000000);
		memset(&drive, 0, sizeof(DRIVE_VALUE));
	
		printf("frame %d\n", frame);
		// Get obstacle information
		memset(&obstacle, 0, sizeof(OBSTACLE_VALUE));
		_RcControl.GetObstacleSensorInfoReq(&obstacle);
		for(int i=0; i < 8; i++){
		    if(obstacle.obstacle[i] == 0)
			{
				//printf("%d=near ", i);
				printf("%d=%dmm ", i, obstacle.obstacle[i]);
			}
		    else if(obstacle.obstacle[i] == 0x1000)
			{
				//printf("%d=long ", i);
				printf("%d=%dmm ", i, obstacle.obstacle[i]);
			}
		    else
			printf("%d=%dmm ", i, obstacle.obstacle[i]);
		}
		printf("\n");


	}

	_RcControl.Stop();
	_RcControl.Close();

   return 0;
}
