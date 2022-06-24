/*
 * ServerApp.h
 *
 *  Created on: 2009/06/23
 *      Author: sekiguchi
 */

#ifndef SERVERAPP_H_
#define SERVERAPP_H_

//#include "IpmManager.h"
#include "RczCommServer.h"
#include "RcControl.h"
#include "LaserRangeSensor.h"

using namespace zmp::zrc;

class ServerApp : public RcControlCommandHandler, LaserRangeSensorReceiveHandler
{
public:
	ServerApp();
	ServerApp(int port_no);
	virtual ~ServerApp();

public:
	bool Init();
	bool Start();
	bool Process();
	bool Stop();
	bool Close();
	bool IsConnect();
	bool GetClientName(char *buff);

	void SetRcControlCommandHandler(RcControlCommandHandler *handler);

	// overrides RczCommSererMessageHandler
	void onSetTorqueEnable(const RczHMsgSetTorqueEnable *msg);
	void onSetSteerAngle(const RczHMsgSetSteerAngle *msg);
	void onSetDriveSpeed(const RczHMsgSetDriveSpeed *msg);
	void onSetControlMode(const RczHMsgSetControlMode *msg);
	void onSetServoTorqueEnable(const RczHMsgSetTorqueEnable *msg);
	void onSetAngleZeroOffset(const RczHMsgSetAngleZeroOffset *msg);
	void onSetLaneKeepingParam(const RczHMsgSetLaneKeepingParam *msg);
	void onGetSteerInfo(RczHMsgGetServoRes *msg);
//	void onGetSensorInfo(RczHMsgGetSensorRes *msg);
//	void onGetObstacleInfo(RczHMsgGetObstacleRes *msg);
//	void onGetPowerInfo(RczHMsgGetPowerRes *msg);
	void onSetServoOnOff(const RczHMsgSetServoOnOff *msg);
	void onSetMotorOnOff(const RczHMsgSetMotorOnOff *msg);
	void onSetGyroOffset(const RczHMsgSetGyroOffset *msg);
	void onSetCurrentOffset(const RczHMsgSetCurrentOffset *msg);
	void onGetCamberInfo(RczHMsgGetCamberRes *msg);
	void onSetCamberOnOff(const RczHMsgSetCamberOnOff *msg);
	void onSetCamberAngle(const RczHMsgSetCamberAngle *msg);
	void onSetSteerOffset(const RczHMsgSetSteerOffset *msg);
	void onSetCamberOffset(const RczHMsgSetCamberOffset *msg);

	void onBeginMotion(const RczHMsgBeginMotion *msg);
	void onSetObstacleAvoidanceParam(const RczHMsgSetObstacleAvoidanceParam *msg);

	void SendServoInfo(int id, DRIVE_VALUE drive);
	void SendSensorInfo(SENSOR_VALUE sensor);
	void SendObstacleInfo(OBSTACLE_VALUE obstacle);
	void SendPowerInfo(POWER_VALUE power);
	void SendThermoInfo(THERMO_VALUE thermo);
	void SendCamberInfo(RczHMsgGetCamberRes camber);
	void SendLRSInfo(LrsResult res, int dev);

private:
	int _port_no;

	int _current_program_id;
	RcControlCommandHandler *_handler_rc;

	uchar _report_image_id;
	RczCommServer *_server;
	RcControl _rcc;
	LaserRangeSensor _lrs[2];
	bool _lrsFlg[2];

	int _lrsCnt[2];

	void OnReceive(int devNo);
	int _rcvChkCnt;
};

#endif /* SERVERAPP_H_ */
