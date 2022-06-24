//============================================================================
// Name        : SampleLRS.cpp
// Author      :
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include "LaserRangeSensor.h"
#include "RcControl.h"
#include <iostream>
#include <string.h>
#include <unistd.h>
#include <signal.h>

using namespace zmp::zrc;
using namespace qrk;
using namespace std;

class SampleLRS :public LaserRangeSensorReceiveHandler {
public:
	SampleLRS(){};
	virtual ~SampleLRS(){};

	bool Init() {
		_rCnt = -1;
		bool res = lrs.InitSerial2();
		if(res == false){
			printf("LRS init ERROR\n");
			return false;
		}
		lrs.SetScanParam(-120, 120, 0, 0);

		if(rrc.init())
		    return true;
		else
		    return false;
	}

	void SetReceiveHandler() {
		lrs.SetReceiveHander(this);
	}

	bool Start(){
		bool res = lrs.Start();
		if(res == false){
			printf("LRS start ERROR\n");
			return false;
		};

		if(!rrc.Start())
		    return false;
		rrc.SetServoEnable(1);
		rrc.SetMotorEnableReq(1);
		rrc.SetDriveSpeed(0);
		rrc.SetSteerAngle(0);
		return true;
	}

	bool Stop(){
		rrc.SetDriveSpeed(0);
		rrc.SetServoEnable(0);
		rrc.SetMotorEnableReq(0);	
		lrs.Stop();
		rrc.Stop();
		return true;
	}

private:
	void OnReceive(int nodeNo){
printf("OnReceive()0\n");
		if(_rCnt > 0) {
printf("OnReceive()1\n");
			_rCnt -= 1;
			return; 
		} else if (_rCnt = 0) {
printf("OnReceive()2\n");
			_rCnt = -1;
//			rrc.SetSteerAngle(_rstAngle*-1);
//			rrc.SetDriveSpeed(_rstSpeed*-1);
			return; 
		}
printf("OnReceive()3 no=%d\n", nodeNo);
		
		if(nodeNo != 2)
			return;	
printf("OnReceive()4\n");
	    float angle = 0;
	    int speed = 280;
	    if(lrs.GetData(&res)){
		for(int i=0; i<15; i++){
		    area[i] = 5600;
		}
		for(int j=44; j < res.data_length; j++){
		    if(res.data[j] <= 20)
			res.data[j] = 5600;
		    switch(j/50){
		    case 0:
			if(area[0] > res.data[j])
			    area[0] = res.data[j];
			break;
		    case 1:
			if(area[1] > res.data[j])
			    area[1] = res.data[j];
			break;
		    case 2:
			if(area[2] > res.data[j])
			    area[2] = res.data[j];
			break;
		    case 3:
			if(area[3] > res.data[j])
			    area[3] = res.data[j];
			break;
		    case 4:
			if(area[4] > res.data[j]);
			    area[4] = res.data[j];
			break;
		    case 5:
			if(area[5] > res.data[j])
			    area[5] = res.data[j];
			break;
		    case 6:
			if(area[6] > res.data[j])
			    area[6] = res.data[j];
			break;
		    case 7:
			if(area[7] > res.data[j])
			    area[7] = res.data[j];
			break;
		    case 8:
			if(area[8] > res.data[j])
			    area[8] = res.data[j];
		    case 9:
			if(area[9] > res.data[j])
			    area[9] = res.data[j];
			break;
		    case 10:
			if(area[10] > res.data[j])
			    area[10] = res.data[j];
			break;
		    case 11:
			if(area[11] > res.data[j])
			    area[11] = res.data[j];
			break;
		    case 12:
			if(area[12] > res.data[j])
			    area[12] = res.data[j];
			break;
		    case 13:
			if(area[13] > res.data[j])
			    area[13] = res.data[j];
			break;
		    case 14:
			if(area[14] > res.data[j])
			    area[14] = res.data[j];
			break;
		    default:
			break;
		    }
		}
		
		angle = 0;
		for(int k=3; k<12; k++){
		    printf("%d=%ld ", k, area[k]);
		    if((area[k-1] > area[k]) && (area[k] < 600)){
			if(k < 8)
			    angle = 30;
			else
			    angle = -30;
		    }
		}

		if((area[3] <= 250) || (area[4] <= 250) || (area[5] <= 250) ||
		   (area[6] <= 250) || (area[7] <= 250) || (area[8] <= 250) ||
		   (area[9] <= 250) || (area[10]<= 250) || (area[11] <= 250)){
		    speed = -280;
		    angle = angle * -1;
		 	_rCnt = 5;	
		}
		else
		    speed = 280;

		printf("\n");
		printf("set angle=%f\n", angle);
		_rstAngle = angle;
		_rstSpeed = speed;
		rrc.SetSteerAngle(angle);
		rrc.SetDriveSpeed(speed*-1);
	    }
	}

	LaserRangeSensor lrs;
	RcControl	rrc;
	long area[15];
	LrsResult res;
	int _rCnt;
    int _rstAngle; 
	int _rstSpeed;
};

void funcx(int sig);
static SampleLRS slrs;
int main() {
//	SampleLRS slrs;

	signal(SIGINT, funcx);

	bool flg = 1;	
	bool ires = slrs.Init();
	if(ires == false)
	    flg = 0;
	slrs.SetReceiveHandler();
	bool sres = slrs.Start();
	if(sres == false)
	    flg = 0;

	while (flg) {
	    usleep(3000);
	}
	slrs.Stop(); // Stop the device and receive thread.
	return 0;
}

void funcx(int sig) {
    slrs.Stop();
	signal(SIGINT, SIG_DFL);
	raise(SIGINT);
}
