/*
 * ServerApp.cpp
 *
 *  Created on: 2009/06/23
 *      Author: sekiguchi
 */

#include "ServerApp.h"

#include <stdlib.h>
#include <string.h>

#define DEFAULT_PORT_NO 9292

ServerApp::ServerApp()
	: _port_no(DEFAULT_PORT_NO)
	, _handler_rc(NULL)
	, _server(NULL)
{
}

ServerApp::ServerApp(int port_no)
	: _port_no(port_no)
	, _handler_rc(NULL)
	, _server(NULL)
{
}

ServerApp::~ServerApp()
{
}

void ServerApp::SetRcControlCommandHandler(RcControlCommandHandler *handler)
{
	_handler_rc = handler;
}

bool ServerApp::Init()
{
	_rcvChkCnt = 0;
	_rcc.init();
	bool res = _lrs[0].InitSerial();
	if(res == false){
		printf("LRS init ERROR \n");
		_lrsFlg[0] = false;
		return false;
	}
	_lrsFlg[0] = true;
	_lrs[0].SetScanParam(-120, 120, 0, 0);
	return true;
}

bool ServerApp::Start()
{
	_server = new RczCommServer();
	_server->Init(_port_no);
	if(_handler_rc)
		_server->SetReceiveHandler(_handler_rc);
	else
		_server->SetReceiveHandler(this);

	if(_lrsFlg[0] == true)
		_lrs[0].SetReceiveHander(this);
	_server->Start();

	_rcc.Start();
	_rcc.SetReportFlagReq(0x0f);
	if(_lrsFlg[0] == true)
		_lrs[0].Start();
	return true;
}

bool ServerApp::Stop(){
	_rcc.SetDriveSpeed(0);
	_rcc.SetServoEnable(0);
	_rcc.SetMotorEnableReq(0);
	_rcc.Stop();
	if(_lrsFlg[0] == true){
		_lrs[0].Stop();
		_lrsFlg[0] = false;
	}
	return true;
}

bool ServerApp::Close(){
	_rcc.Close();
	return true;
}

void ServerApp::onSetTorqueEnable(const RczHMsgSetTorqueEnable *msg){
	_rcc.SetMotorEnableReq(msg->enable);
}

void ServerApp::onSetControlMode(const RczHMsgSetControlMode *msg){
}

void ServerApp::onSetLaneKeepingParam(const RczHMsgSetLaneKeepingParam *msg){
}

void ServerApp::onSetSteerAngle(const RczHMsgSetSteerAngle *msg){
//	printf("SetSteerAngle() val=%f\n", msg->angle);
	_rcc.SetSteerAngle(msg->angle);
}

void ServerApp::onSetDriveSpeed(const RczHMsgSetDriveSpeed *msg){
//	printf("SetDriveSpeed() val=%d\n", msg->speed);
	_rcvChkCnt = 0;
	_rcc.SetDriveSpeed(msg->speed);
}

void ServerApp::onSetServoTorqueEnable(const RczHMsgSetTorqueEnable *msg){
//	printf("SetServoTorqueEnable() val=%d\n", msg->enable);
	_rcc.SetMotorEnableReq(msg->enable);
}

void ServerApp::onSetAngleZeroOffset(const RczHMsgSetAngleZeroOffset *msg){
//	printf("SetAngleZeroOffset() val=%d\n", (int)(msg->offset));
	_rcc.SetSteerAngleOffset(msg->offset);
}

void ServerApp::onGetSteerInfo(RczHMsgGetServoRes *msg){
//	printf("GetSteerInfo()\n");
	DRIVE_VALUE drive;
	_rcc.GetServoInfoReq(1, PRESENT_POSITION_L, PRESENT_VOLTS_H - PRESENT_POSITION_L, &drive);
	printf("p=%fdeg c=%dmA t=%d v=%dmV\n", 
		drive.present_position, drive.present_current,
		drive.present_temperature, drive.present_volts);
}

void ServerApp::onSetServoOnOff(const RczHMsgSetServoOnOff *msg)
{
	_rcc.SetServoEnable(msg->onOff);
}

void ServerApp::onSetMotorOnOff(const RczHMsgSetMotorOnOff *msg)
{
	_rcc.SetMotorEnableReq(msg->onOff);
}

void ServerApp::onSetGyroOffset(const RczHMsgSetGyroOffset *msg)
{
	_rcc.SetGyroOffset(msg->offset);
}

void ServerApp::onSetCurrentOffset(const RczHMsgSetCurrentOffset *msg)
{
	_rcc.SetMotorCurrentOffset(msg->offset);
}

void ServerApp::onSetCamberOnOff(const RczHMsgSetCamberOnOff *msg)
{
	_rcc.SetCamberEnable(msg->servo_id, msg->onOff);
}

void ServerApp::onSetCamberAngle(const RczHMsgSetCamberAngle *msg)
{
//	printf("set_id=%d set_angle=%f\n", msg->servo_id, msg->angle);
	_rcc.SetCamberAngle(msg->servo_id, msg->angle);
}

void ServerApp::onSetSteerOffset(const RczHMsgSetSteerOffset *msg)
{
printf("ServerApp::onSetSteerOffset()\n");
	_rcc.SetSteerAngleOffset(msg->offset);
}

void ServerApp::onSetCamberOffset(const RczHMsgSetCamberOffset *msg)
{
	_rcc.SetCamberOffset(msg->servo_id, msg->offset);
}

void ServerApp::onBeginMotion(const RczHMsgBeginMotion *msg)
{
}

void ServerApp::onSetObstacleAvoidanceParam(const RczHMsgSetObstacleAvoidanceParam *msg)
{
}

void ServerApp::SendServoInfo(int id, DRIVE_VALUE drive)
{
	RczHMsgGetServoRes msg;
	msg.header[0]		= 0xac;
	msg.header[1]		= 0xca;
	msg.length		= sizeof(RczHMsgGetServoRes);
	msg.msg_id		= RCZHMSG_ID_GET_SERVO_INFO_RES;
	msg.servo_id		= id;
	msg.present_position	= drive.present_position;
	msg.present_speed	= drive.present_speed;
	msg.present_current	= drive.present_current;
	msg.present_temperature = drive.present_temperature;
	msg.present_volts	= drive.present_volts;

//	printf("id=%d pos=%f\n", msg.servo_id, msg.present_position);
	_server->Send((const uchar*)&msg, msg.length); 
}

void ServerApp::SendCamberInfo(RczHMsgGetCamberRes camber)
{
    RczHMsgGetCamberRes msg;
    msg.header[0]	= 0xac;
    msg.header[1]	= 0xca;
    msg.length		= sizeof(RczHMsgGetCamberRes);
    msg.msg_id		= RCZHMSG_ID_GET_CAMBER_INFO_RES;
    for(int i=0; i < 5; i++){
	msg.camber_data[i].present_position	= camber.camber_data[i].present_position;
	msg.camber_data[i].present_speed	= camber.camber_data[i].present_speed;
	msg.camber_data[i].present_current	= camber.camber_data[i].present_current;
	msg.camber_data[i].present_temperature	= camber.camber_data[i].present_temperature;
	msg.camber_data[i].present_volts	= camber.camber_data[i].present_volts;
    }

    _server->Send((const uchar*)&msg, msg.length);
}

void ServerApp::SendSensorInfo(SENSOR_VALUE sensor)
{
	RczHMsgGetSensorRes msg;
	msg.header[0]	= 0xac;
	msg.header[1]	= 0xca;
	msg.length	= sizeof(RczHMsgGetSensorRes);
	msg.msg_id	= RCZHMSG_ID_GET_SENSOR_INFO_RES;
	msg.gyro	= sensor.gyro;
	msg.acc_x	= sensor.acc_x;
	msg.acc_y	= sensor.acc_y;
	msg.acc_z	= sensor.acc_z;
	msg.motor_enc	= sensor.enc_0;
	msg.wheel_enc1	= sensor.enc_1;
	msg.wheel_enc2	= sensor.enc_2;
	msg.wheel_enc3	= sensor.enc_3;
	msg.wheel_enc4	= sensor.enc_4;
	_server->Send((const uchar*)&msg, msg.length);
}

void ServerApp::SendObstacleInfo(OBSTACLE_VALUE obstacle)
{
	RczHMsgGetObstacleRes msg;
	msg.header[0]	= 0xac;
	msg.header[1]	= 0xca;
	msg.length	= sizeof(RczHMsgGetObstacleRes);
	msg.msg_id	= RCZHMSG_ID_GET_OBSTACLE_INFO_RES;
	msg.ir_obstacle[0]	= obstacle.obstacle[0];
	msg.ir_obstacle[1] = obstacle.obstacle[1];
	msg.ir_obstacle[2] = obstacle.obstacle[2];
	msg.ir_obstacle[3] = obstacle.obstacle[3];
	msg.ir_obstacle[4] = obstacle.obstacle[4];
	msg.ir_obstacle[5] = obstacle.obstacle[5];
	msg.ir_obstacle[6] = obstacle.obstacle[6];
	msg.ir_obstacle[7] = obstacle.obstacle[7];

	_server->Send((const uchar*)&msg, msg.length);	
}

void ServerApp::SendThermoInfo(THERMO_VALUE thermo)
{
    RczHMsgGetThermoRes msg;
    msg.header[0]	= 0xac;
    msg.header[1]	= 0xca;
    msg.length		= sizeof(RczHMsgGetThermoRes);
    msg.msg_id		= RCZHMSG_ID_GET_THERMO_INFO_RES;
    msg.thermo_fet1	= thermo.fet1;
    msg.thermo_fet2	= thermo.fet2;
    msg.thermo_motor	= thermo.motor;

    _server->Send((const uchar*)&msg, msg.length);
}

void ServerApp::SendPowerInfo(POWER_VALUE power)
{
	RczHMsgGetPowerRes msg;
	msg.header[0]		= 0xac;
	msg.header[1]		= 0xca;
	msg.length		= sizeof(RczHMsgGetPowerRes);
	msg.msg_id		= RCZHMSG_ID_GET_POWER_INFO_RES;
	msg.motor_current	= power.motor_current;
	msg.motor_battery	= power.battery_level;

	_server->Send((const uchar*)&msg, msg.length);
}

void ServerApp::SendLRSInfo(LrsResult res)
{
	RczHMsgGetLRSRes msg;
	msg.header[0]		= 0xac;
	msg.header[1]		= 0xca;
	msg.length		= sizeof(RczHMsgGetLRSRes);
	msg.msg_id		= RCZHMSG_ID_GET_LRS_INFO_RES;
	msg.lrs_data.magic = res.magic;
	msg.lrs_data.timestamp; res.timestamp;
	msg.lrs_data.data_length = res.data_length/2;
	msg.lrs_data.rcvCnt = res.rcvCnt;
	for(int i=0; i<msg.lrs_data.data_length; i++){
		msg.lrs_data.data[i] = res.data[i*2];
	}	
//	memcpy(&msg.lrs_data, &res, sizeof(LrsResult));
//printf("%d\n", msg.lrs_data.rcvCnt);
	_server->Send((const uchar*)&msg, msg.length);
}

// for measurement
#define CPU_CLOCK_PER_USEC 500.0 //[MHz]=[clock/usec]
#define CPU_USEC_PER_CLOCK 0.002 //[usec/clock]

inline unsigned long long int rdtsc(void)
{
	unsigned long long int x;
	__asm__ volatile (".byte 0x0f, 0x31" : "=A" (x));
	return x;
}

bool ServerApp::IsConnect()
{
	return _server->IsConnect();
}

bool ServerApp::GetClientName(char *buff)
{
	return _server->GetClientName(buff);
}

bool ServerApp::Process()
{
//  unsigned long long prv_tclock0 = 0;
    DRIVE_VALUE drive;
    SENSOR_VALUE sensor;
    OBSTACLE_VALUE obstacle;
    POWER_VALUE power;
    THERMO_VALUE thermo;
//    LrsResult lrs;
//    static unsigned long long prv_tclock0 = 0;
    while (_server->IsReceiveMessage()) {
        _server->KickCallback();
    }
	_rcvChkCnt += 1;
printf("cnt=%d\n", _rcvChkCnt);
	if(_rcvChkCnt >= 50){
		_rcc.SetDriveSpeed(0);
		_rcvChkCnt = 0;
	}
//    usleep(50000);
    _rcc.GetServoInfoReq(1, PRESENT_POSITION_L, PRESENT_VOLTS_H - PRESENT_POSITION_L, &drive);
    SendServoInfo(1, drive);

    _rcc.GetSensorInfoReq(&sensor);
    SendSensorInfo(sensor);

    _rcc.GetObstacleSensorInfoReq(&obstacle);
    SendObstacleInfo(obstacle);

    _rcc.GetPowerInfoReq(&power);
    SendPowerInfo(power);

    _rcc.GetThermoInfoReq(&thermo);
    SendThermoInfo(thermo);
//    unsigned long long tclock0 = rdtsc();
//    double time_usec = ((tclock0 - prv_tclock0) * CPU_USEC_PER_CLOCK + 0.5);
//    printf("proc. %6.2lf[ms]\n", time_usec / 1000.0);
//    prv_tclock0 = tclock0;

//    collect_proc();

    return true;
}

void ServerApp::OnReceive(int dev)
{
	LrsResult lrs;
	if(dev != 1)
		return;
	if(_lrs[0].GetData(&lrs))
			SendLRSInfo(lrs);
}

