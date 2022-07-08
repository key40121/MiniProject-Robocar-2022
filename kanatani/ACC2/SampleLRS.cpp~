#include "LaserRangeSensor.h"
#include "RcControl.h"
#include <iostream>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <stdio.h>
#include <math.h>
#include <sched.h>
#include <sys/types.h>
#include <stdlib.h>
#include <memory.h>
#define MAX_LENGTH 4000    //Laser Range Sensor can measure 4000 [mm] at most
#define MIN_LENGTH 40      //Laser Range Sensor can't measure under 40 [mm]

using namespace zmp::zrc;
using namespace qrk;
using namespace std;

RcControl  _RcControl;
void funcx(int sing);

int main()
{
	signal(SIGINT, funcx);
    _RcControl.init();
    _RcControl.Start(); 
    _RcControl.SetReportFlagReq(0x0f);
    _RcControl.SetServoEnable(1);
    _RcControl.SetMotorEnableReq(1);
    _RcControl.SetDriveSpeed(0);
    _RcControl.SetSteerAngle(0);

	RcControl	rrc;
	LaserRangeSensor _lrs[2];
	LrsResult res;
	LrsResult lrs;

	bool _lrsFlg[2];
	int min_dist;
	float angle;
	int set_speed;
	int j;
	bool loop = true;
	res = _lrs[0].InitSerial();
	

	std::cout << "set_speed >>";
	std::cin >> set_speed;

	while(loop)
	{
		min_dist = 4000;
		for(j= 255; j< 428; j ++)
		{
			if(min_dist > res.data[j])
			{
				min_dist = res.data[j];
				angle = -120+240.0/res.data_length*j;
			}
			printf("min = %d, angle = %f\n",min_dist, angle);
			std::cout << min_dist << " " << angle << "\n";
/*
			if((min_dist >= 200) && (min_dist < set_speed*8/5+200))
			{
				_RcControl.SetDriveSpeed((min_dist-200)*5/8);
				_RcControl.SetSteerAngle(angle);
			}

			else if(min_dist >= set_speed*8/5+200)
			{
				_RcControl.SetDriveSpeed(set_speed);
				_RcControl.SetSteerAngle(0);
			}

			else
			{
				_RcControl.SetDriveSpeed(0);
				_RcControl.SetSteerAngle(0);
			}*/
		}
	}
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
