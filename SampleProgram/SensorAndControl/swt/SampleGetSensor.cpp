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

	while(1){
		usleep(10000);
		memset(&drive, 0, sizeof(DRIVE_VALUE));
		_RcControl.GetServoInfoReq(1, PRESENT_POSITION_L, PRESENT_VOLTS_H - PRESENT_POSITION_L, &drive);
		printf("angle=%fdeg, current=%dmA, temp=%d℃, volts=%dmV\n",
				drive.present_position,
				drive.present_current,
				drive.present_temperature,
				drive.present_volts);

		// Get sensor information
		memset(&sensor, 0, sizeof(SENSOR_VALUE));
		_RcControl.GetSensorInfoReq(&sensor);
		printf("G=%3.2fdeg/s, ax=%3.2fg, ay=%3.2fg, az=%3.2fg, e0=%3.2fcycle/s, e1=%3.0fmm/s, e2=%3.0fmm/s, e3=%3.0fmm/s, e4=%3.0fmm/s\n",
				sensor.gyro,
				sensor.acc_x,
				sensor.acc_y,
				sensor.acc_z,
				sensor.enc_0,
				sensor.enc_1,
				sensor.enc_2,
				sensor.enc_3,
				sensor.enc_4);

		// Get obstacle information
		memset(&obstacle, 0, sizeof(OBSTACLE_VALUE));
		_RcControl.GetObstacleSensorInfoReq(&obstacle);
		for(int i=0; i < 8; i++){
		    if(obstacle.obstacle[i] == 0)
			printf("%d=near ", i);
		    else if(obstacle.obstacle[i] == 0x1000)
			printf("%d=long ", i);
		    else
			printf("%d=%dmm ", i, obstacle.obstacle[i]);
		}
		printf("\n");

		// Get power information
		memset(&power, 0, sizeof(POWER_VALUE));
		_RcControl.GetPowerInfoReq(&power);
		printf("current=%2.2fmA, battery=%2.2fV\n",
			power.motor_current, power.battery_level);

		memset(&thermo, 0, sizeof(THERMO_VALUE));
		_RcControl.GetThermoInfoReq(&thermo);
		printf("fet1=%2.2f, fet2=%2.2f, motor=%2.2f\n", 
			thermo.fet1, thermo.fet2, thermo.motor);
	}

	_RcControl.Stop();
	_RcControl.Close();

   return 0;
}
