//============================================================================
// Name        : SampleLRS.cpp
// Author      : Tetsuya Azuma
// Version     : 1.0.0
// Copyright   :
// Description : Acquiring data from Laser Range sensor
//============================================================================

#include <iostream>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <stdio.h>
#include <math.h>
#include "LaserRangeSensor.h"
#include "RcControl.h"

#define MAX_LENGTH 4000    //Laser Range Sensor can measure 4000 [mm] at most
#define MIN_LENGTH 40      //Laser Range Sensor can't measure under 40 [mm]

using namespace zmp::zrc;
using namespace qrk;
using namespace std;

class SampleLRS :public LaserRangeSensorReceiveHandler {
public:
	SampleLRS(){};
	virtual ~SampleLRS(){};

	bool Init() {
		bool res = _lrs[0].InitSerial();
		if(res != true){
			printf("[LRS info] Rear LRS : Initialize error\n");
			_lrsFlg[0] = false;
			return false;
		}
		_lrsFlg[0] = true;
		_lrs[0].SetScanParam(-120, 120, 0, 0);

		res = _lrs[1].InitSerial2();
		if(res != true){
			printf("[LRS info] Front LRS : Initialize error\n");
			_lrsFlg[1] = false;
			return false;
		}
		_lrsFlg[1] = true;
		_lrs[1].SetScanParam(-120, 120, 0, 0);
		printf("[LRS info] Initialized\n");

		if(rrc.init()){
		    printf("[rrc info] Initialized\n");
		    return true;
		}else{
		    return false;
		}
	}

	bool Start(){
		if(_lrsFlg[0] == true){
			_lrs[0].SetReceiveHander(this);
			bool res = _lrs[0].Start();
			if(res != true){
				printf("[LRS info] Rear LRS : Failed to start\n");
			return false;
			};
			printf("[LRS info] Rear LRS : Success to start\n");
		}
		if(_lrsFlg[1] == true){
			_lrs[1].SetReceiveHander(this);
			bool res = _lrs[1].Start();
			if(res != true){
				printf("[LRS info] Front LRS : Failed to start\n");
				return false;
			};
			printf("[LRS info] Front LRS : Success to start\n");
		}
		return true;
	}

	bool Stop(){
		bool ret[2];
		do{
			ret[0] = _lrs[0].Stop();
			ret[1] = _lrs[1].Stop();
		}while(ret[0] && ret[1]);
		printf("[LRS info] both LRS : Success to stop\n");
		rrc.Stop();
		return true;
	}

private:

	//_/_/_/_/_/ / Read Point cloud data_/_ /_/_/_/_/_//
	void storeLRF(LrsResult res, int dev){
		if(dev == 1){
			num = 0;
			max_dist[0] = 0;
			min_dist[0] = 4000;
			for( int j= 44; j< res.data_length; j ++){

					cout << res.data[j] << endl;

				if( MIN_LENGTH < res.data[j] && res.data[j] < MAX_LENGTH ){

					
					dist[0][num] = res.data[j];
					num ++;
					if(max_dist[0] < res.data[j])
						max_dist[0] = res.data[j];
					if(min_dist[0] > res.data[j])
						min_dist[0] = res.data[j];
				}
			}
			printf("[rear data info] max = %d, min = %d, count = %d\n",max_dist[0],min_dist[0],num);
			// cout << res.data_length << endl;


		} else {
			num = 0;
			max_dist[1] = 0;
			min_dist[1] = 4000;
			for( int j= 44; j< res.data_length; j ++){
				if( MIN_LENGTH < res.data[j] && res.data[j] < MAX_LENGTH){
				dist[1][num] = res.data[j];
					num ++;
					if(max_dist[1] < res.data[j])
						max_dist[1] = res.data[j];
					if(min_dist[1] > res.data[j])
						min_dist[1] = res.data[j];
				}
			}
			printf("[front data info] max = %d, min = %d, count = %d\n",max_dist[1],min_dist[1],num);
		}
	}

	//_/_/_/_/_/ Callback function_/_/_/_/_/_/_//
	void OnReceive(int dev){
	    LrsResult lrs;

	    if(dev == 1){
	    	if(_lrs[0].GetData(&lrs)){
			storeLRF(lrs, dev);
			//printf("recieve front data\n");
		}
	    } else {
		if(_lrs[1].GetData(&lrs)){
			storeLRF(lrs, dev);
			//printf("recieve rear data\n");
		}
	    }
	}

	//_/_/_/_/_/ Variable declaration_/_ /_/_/_/_/_//

	RcControl	rrc;
	LaserRangeSensor _lrs[2];
	bool _lrsFlg[2];
	int dist[2][1400];
	int max_dist[2];
	int min_dist[2];
	int num;
};

void funcx(int sig);
static SampleLRS slrs;
int main() {

	signal(SIGINT, funcx);

	bool flg = 1;
	bool ires = slrs.Init();
	if(ires != true)
	    flg = 0;

	bool sres = slrs.Start();
	if(sres != true)
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
