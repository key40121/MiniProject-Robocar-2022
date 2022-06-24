/**
 * @file
 * @~english
 * @brief	Laser range sensor class header.
 * @author	Masaki SegaWa
 * @~japanese
 * @brief	レーザーレンジセンサクラスヘッダ.
 * @author	瀬川正樹
 * @~
 * @date	2009-03-26
 * Copyright (c) 2009 ZMP Inc. All rights reserved.
 */

#ifndef LASERRANGESENSOR_H_
#define LASERRANGESENSOR_H_

#include <pthread.h>
//extern "C" {
//#include "urg_ctrl.h"
//}
#include "Urg_driver.h"
#include <iostream>

using namespace qrk;
using namespace std;

namespace zmp {
	namespace zrc {


class CRingBuffer;

/**
 * @~english
 * Exception of Ipm operations.
 * It throws by methods of the IpmControl class.
 * @~japanese
 * IPM操作の例外.
 * IpmControlのメソッドの例外として投げられる。
 */
class LrsException
{
public:
	/**
	 * @~english
	 * The result code of operation of Laser range sensor.
	 * @~japanese
	 * レーザーレンジセンサのエラーコード.
	 */
	enum LRS_RESULT_CODE {
		SUCCESS,
		CAN_NOT_OPEN_DEVICE,
		SET_PARAMETER_FAILED,
		REQUEST_DATA_FAILED,
		NOT_SUPPORTED,
	};
};






/**
 * @~english
 * Laser range sensor message handler interface.
 * This is pure virtual class. It provides the callback method of receiving data from the LRS.
 * @~japanese
 * レーザーレンジセンサメッセージハンドラインターフェイス
 * 仮想クラス。LRSからのデータ受信時のコールバックメソッドを提供する。
 */
class LaserRangeSensorReceiveHandler {
public:
	virtual void OnReceive(int devNo) = 0;
	//virtual void OnError() = 0;
};

#define LSR_RESULT_MAX_DATA_LENGTH 726 

struct LrsResult {
	unsigned long magic;
	long timestamp;
	int data_length;
	int rcvCnt;	
//	float	angle[LSR_RESULT_MAX_DATA_LENGTH];
	unsigned short	data[LSR_RESULT_MAX_DATA_LENGTH];
};


class LaserRangeSensor
{

public:
	LaserRangeSensor();
	LaserRangeSensor(int buffer_depth);
	virtual ~LaserRangeSensor();

public:
	bool InitSerial();
	bool InitSerial2();
	bool InitEther(char* ip);

    bool SetScanParam(int minAngle, int maxAngle, int skipStep, int skipScan);  

	void SetReceiveHander(LaserRangeSensorReceiveHandler *handler);
	bool Start();
	bool Stop();
	bool GetData(LrsResult *res);

    double index2rad(int index);
    double index2deg(int index);
    int rad2index(double radian);
    int deg2index(double degree);
    int rad2step(double radian);
	int deg2step(double degree);
	double step2rad(int step);
	double step2deg(int step);
	int step2index(int step);

	int min_step();
	int max_step();
	long min_distance();
	long max_distance();
	long scan_usec();
	int max_data_size();
	int max_echo_size();

	char* product_type();
	char* firmware_version();
	char* serial_id();
	char* status();
	char* state();

protected:
	void ReceiveThread();
	static void *ReceiveThreadEntry(void *arg);

	bool connectDevice();
	bool setupDevice();
	int getLengthDevice();
	void disconnectDevice();

private:
	Urg_driver		*_urg;
	CRingBuffer		*_rcvBuffer;
	int 			_data_length_a_scan;
	int				_buffer_depth;
	LrsException::LRS_RESULT_CODE _err_code;

	pthread_t 		_receive_thread;
	bool 			_b_active_thread;
	bool			_b_connect;
	bool			_b_req_stop_thread;

	static const int 	_DEFAULT_BUFFER_DEPTH;
	static const char 	*_DEVICE_NAME;
	static const long 	_BAUDRATE;
	LaserRangeSensorReceiveHandler	*_callback_handler;

	const char* _device_or_ip;
	int			_rate_or_port;
	int			_firstAngle;
	int			_lastAngle;
	int			_skipStep;
	int 		_skipScan;
	Lidar::connection_type_t _type;
	int			_rcvCnt;
	int			_devNo;
};




	} // namespace zrc
} // namespace zmp

#endif /* RASERRANGESENSOR_H_ */
