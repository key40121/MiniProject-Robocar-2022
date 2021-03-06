 //============================================================================
// Name        : SampleDriveControl.cpp
// Author      : Koji Sekiguchi
// Version     : 1.0.0
// Copyright   :
// Description : Control rotational speed and angle
// Copyright (c) 2009 ZMP Inc. All rights reserved
//============================================================================

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

int main(){
  	
	int distance = 600;
	int distance1 = 400;
	int distance2 = 300;
	int speedset = -50;
	int angleset = 30;

	OBSTACLE_VALUE obstacle;
    char buf[20];
    float angle = 0;
    int speed = 0;
    int set_v = 0;
    int set_t = 0;
    bool sign = false;
    float set_a = 0;
    bool period = false;
    bool loop = true;
    RcControl  _RcControl;
    _RcControl.init();
    _RcControl.Start();
 
    _RcControl.SetReportFlagReq(0x0f);
    _RcControl.SetServoEnable(1);
    _RcControl.SetMotorEnableReq(1);
    _RcControl.SetDriveSpeed(0);
    _RcControl.SetSteerAngle(0);


    while(loop){
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

		if(obstacle.obstacle[0] <= distance2 || obstacle.obstacle[1] <= distance1 || obstacle.obstacle[2] <= distance && obstacle.obstacle[3] > distance1 && obstacle.obstacle[4] > distance2)
		{	
			angle = -angleset;
			speed = speedset;

		}
		else if(obstacle.obstacle[4] <= distance2 || obstacle.obstacle[3] <= distance1 || obstacle.obstacle[2] <= distance && obstacle.obstacle[1] > distance1 && obstacle.obstacle[0] > distance2)
		{	
			angle = angleset;
			speed = speedset;
		}
		else if(obstacle.obstacle[4] <= distance2 &&  obstacle.obstacle[3] <= distance1 && obstacle.obstacle[2] <= distance && obstacle.obstacle[1] <= distance1 && obstacle.obstacle[0] <= distance2)
		{	
			angle = angleset;
			speed = -speedset;
		}
		else
		{
			angle = 0;
			speed = speedset;
		}	
		_RcControl.SetSteerAngle(angle);
		_RcControl.SetDriveSpeed(speed);
		usleep(100);

        
    }
	_RcControl.SetSteerAngle(0);
	_RcControl.SetDriveSpeed(0);
    _RcControl.Stop();
    _RcControl.Close();
    printf("loop end\n");
    return 0;
}
