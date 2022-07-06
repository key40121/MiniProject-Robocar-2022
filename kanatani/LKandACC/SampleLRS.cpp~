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
int set_speed;

class SampleLRS :public LaserRangeSensorReceiveHandler {
public:
	SampleLRS(){};
	virtual ~SampleLRS(){};

	bool Init() {
		bool res = _lrs[0].InitSerial();

		if(res != true)
		{
			_lrsFlg[0] = false;
			return false;
		}
		_lrsFlg[0] = true;
		_lrs[0].SetScanParam(-120, 120, 0, 0);
		res = _lrs[1].InitSerial2();

		if(res != true)
		{
			_lrsFlg[1] = false;
			return false;
		}
		_lrsFlg[1] = true;
		_lrs[1].SetScanParam(-120, 120, 0, 0);
		
		if(rrc.init())
		{
			return true;
		}
		else
		{
			return false;
		}
	}

	bool Start(){
		if(_lrsFlg[0] == true)
		{
			_lrs[0].SetReceiveHander(this);
			bool res = _lrs[0].Start();
		
			if(res != true)
			{
				return false;
			};
		}

		if(_lrsFlg[1] == true)
		{
			_lrs[1].SetReceiveHander(this);
			bool res = _lrs[1].Start();

			if(res != true)
			{
				return false;
			};
		}
		return true;
	}

	bool Stop(){
		bool ret[2];
		do
		{
			ret[0] = _lrs[0].Stop();
			ret[1] = _lrs[1].Stop();
		}
		while(ret[0] && ret[1]);

		rrc.Stop();
		return true;
	}

private:

	//_/_/_/_/_/ / Read Point cloud data_/_ /_/_/_/_/_//
	void storeLRF(LrsResult res, int dev){

		if(dev == 1){
			
			min_dist = 4000;
			for( int j= 255; j< 428; j ++)
			{
				//std::cout << res.data[j] << " " << -120+240.0/res.data_length*j << "\n";

				if(min_dist > res.data[j])
				{
					min_dist = res.data[j];
					angle = -120+240.0/res.data_length*j;
				}
			}
			std::cout << min_dist << " " << angle << "\n";
			_RcControl.SetDriveSpeed(set_speed);
			_RcControl.SetSteerAngle(angle);

			if((min_dist >= 200) && (min_dist < set_speed*8/5+200))
			{
				_RcControl.SetDriveSpeed((min_dist-200)*5/8);
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
			}
		} 
	}

	//_/_/_/_/_/ Callback function_/_/_/_/_/_/_//
	void OnReceive(int dev){
		LrsResult lrs;

		if(dev == 1){
			if(_lrs[0].GetData(&lrs)){
			storeLRF(lrs, dev);
			}
		}
		else
		{
			if(_lrs[1].GetData(&lrs)){
				storeLRF(lrs, dev);
			}
		}
	}
	
	//_/_/_/_/_/ Variable declaration_/_ /_/_/_/_/_//

	RcControl	rrc;
	LaserRangeSensor _lrs[2];
	bool _lrsFlg[2];
	int min_dist;
	float angle;
};

void funcx(int sig);
static SampleLRS slrs;
int main() {
	signal(SIGINT, funcx);

    _RcControl.init();
    _RcControl.Start();

    _RcControl.SetReportFlagReq(0x0f);
    _RcControl.SetServoEnable(1);
    _RcControl.SetMotorEnableReq(1);
    _RcControl.SetDriveSpeed(0);
    _RcControl.SetSteerAngle(0);	
	std::cout << "set_speed >>";
	std::cin >> set_speed;
	bool flg = 1;
	bool ires = slrs.Init();
	if(ires != true)
	{
		flg = 0;
	}
	
	bool sres = slrs.Start();
	if(sres != true)
	{
		flg = 0;
	}

	while (flg)
	{
		usleep(30000);
	}
	slrs.Stop(); // Stop the device and receive thread.
	return 0;
}

void funcx(int sig) {

    slrs.Stop();
	_RcControl.SetDriveSpeed(0);
    _RcControl.SetSteerAngle(0);
	_RcControl.SetServoEnable(0);
    _RcControl.SetMotorEnableReq(0);
    signal(SIGINT, SIG_DFL);
    raise(SIGINT);
}
