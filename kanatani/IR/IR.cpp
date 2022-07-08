 //============================================================================
// Name        : SampleDriveControl.cpp
// Author      : Koji Sekiguchi
// Version     : 1.0.0
// Copyright   :
// Description : Control rotational speed and angle
// Copyright (c) 2009 ZMP Inc. All rights reserved
//============================================================================

#include <signal.h>
#include <iostream>
#include <termios.h>
#include <fcntl.h>
#include <strings.h>
#include <string.h>
#include <unistd.h>

#include <sched.h>
#include <sys/types.h>
#include <stdio.h>
#include <stdlib.h>
#include <memory.h>
#include "RcControl.h"

using namespace zmp::zrc;
void funcx(int sig);
RcControl  _RcControl;

int main(){
	OBSTACLE_VALUE obstacle;
    char buf[20];
    float angle = 0;
    int speed = 200;
    int set_v = 0;
    int set_t = 0;
    bool sign = false;
    float set_a = 0;
    bool period = false;
    bool loop = true;
	int flg = 0;
	int flg2 = 0;
	int cunt = 0;

	signal(SIGINT, funcx);
    _RcControl.init();
    _RcControl.Start();
 
    _RcControl.SetReportFlagReq(0x0f);
    _RcControl.SetServoEnable(1);
    _RcControl.SetMotorEnableReq(1);
    _RcControl.SetDriveSpeed(speed);
    _RcControl.SetSteerAngle(angle);


    while(loop){
		memset(&obstacle, 0, sizeof(OBSTACLE_VALUE));
		_RcControl.GetObstacleSensorInfoReq(&obstacle);
		if(flg == 0 && obstacle.obstacle[5] >= 500)
			cunt += 1;
		else
			cunt = 0;
		if(cunt >= 15000000)
		{
			_RcControl.SetSteerAngle(30);
			_RcControl.SetDriveSpeed(200);
			sleep(2);	
			_RcControl.SetDriveSpeed(0);
			cunt = 0;
			flg = 1;		
		}
		if(flg == 1)
		{
			if(flg2 == 0){
				_RcControl.SetSteerAngle(-30);
				_RcControl.SetDriveSpeed(-100);
			}
			if(flg2 == 1)
			{
				_RcControl.SetSteerAngle(0);
				_RcControl.SetDriveSpeed(-100);			
			}

			if(obstacle.obstacle[2] <= 100)
			{	
				_RcControl.SetSteerAngle(0);
				_RcControl.SetDriveSpeed(0);
				flg = 2;
			}
			else if(obstacle.obstacle[0] <= 100)
			{				
				_RcControl.SetSteerAngle(30);
				_RcControl.SetDriveSpeed(100);
				sleep(3);
				flg2 = 1;
			}
			else if(obstacle.obstacle[4] <= 100)
			{			
				_RcControl.SetSteerAngle(-30);
				_RcControl.SetDriveSpeed(100);
				sleep(3);
				flg2 = 1;
			}
			}
		
        
    }
	_RcControl.SetSteerAngle(0);
	_RcControl.SetDriveSpeed(0);
    _RcControl.Stop();
    _RcControl.Close();
    //printf("loop end\n");
    return 0;
}

void funcx(int sig) {
	_RcControl.SetDriveSpeed(0);
    _RcControl.SetSteerAngle(0);
	_RcControl.SetServoEnable(0);
    _RcControl.SetMotorEnableReq(0);
    signal(SIGINT, SIG_DFL);
    raise(SIGINT);
}
