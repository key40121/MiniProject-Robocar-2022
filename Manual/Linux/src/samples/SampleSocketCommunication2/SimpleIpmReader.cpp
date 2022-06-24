/**
 * @file
 *
 * @~english
 * @brief	Communication class header.
 *
 * @author	Masaki SegaWa
 *
 * @~japanese
 * @brief	通信クラスの実装.
 *
 * @author	瀬川正樹
 *
 * @~
 * @date	2009-06-19
 *
 * Copyright (c) 2009 ZMP Inc. All rights reserved.
 *
 */

#include "SimpleIpmReader.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>


SimpleIpmReader::SimpleIpmReader()
	: _ipmm(NULL)
	, _length(SIMPLE_IMAGE_SIZE)
{
	_data = new uchar[SIMPLE_IMAGE_SIZE];
}

SimpleIpmReader::~SimpleIpmReader()
{
	delete[] _data;
}

bool SimpleIpmReader::Init()
{
	pthread_mutex_init(&_mutex, NULL);
	
	_ipmm = new IpmManager();
	if (!_ipmm->Init()) {
		return false;
	}
	_ipmm->StopImap();

	return true;
}

bool SimpleIpmReader::Start()
{

        if (!_ipmm->Reset()) {
                printf("Reset() failed.\n");
                goto err;
        }
        if (!_ipmm->LoadProgram(IMAP_ALG_STEREO_OBSTACLES_1)) {
                printf("LoadProgram() failed.\n");
                goto err;
        }
        if (!_ipmm->StartImap()) {
                printf("StartImap() failed.\n");
                goto err;
        }
        if (!_ipmm->SelectImageOutput(SOF_INPUT_L_HALF)) {
                printf("SelectImageOutput() failed.\n");
                goto err;
        }
#if 0
        if (!_ipmm->SelectResultOutput(SOR_RESULT_HIST)) {
                printf("SelectResultOutput() failed.\n");
                goto err;
        }
        if (!_ipmm->SetParam(param_set)) {
                printf("SetParam() failed.\n");
                goto err;
        }
#endif

	if (pthread_create(&_th, NULL, ThreadEntry, this) != 0) {
                printf("pthread_create failed.\n");
		goto err;
	}

	return true;
err:
	return false;
}

void *SimpleIpmReader::ThreadEntry(void *arg)
{
	SimpleIpmReader *obj = (SimpleIpmReader*)arg;
	obj->Thread();
	return 0;
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

void SimpleIpmReader::Thread()
{
	printf("Thread Started()\n");

	static unsigned long long prv_tclock0 = 0;

	while (1) {
		unsigned long long tclock0 = rdtsc();
//		double time_usec = ((tclock0 - prv_tclock0) * CPU_USEC_PER_CLOCK + 0.5);
//		printf("proc. %6.2lf[ms]\n", time_usec / 1000.0);
		prv_tclock0 = tclock0;
	
		collect_proc();
	}
}

bool SimpleIpmReader::collect_proc()
{
	_ipmm->Wait();
#if 0
	if (_ipmm->CollectResult()) {
		int result_flag = _ipmm->GetCurrentResultOutput();
		if (result_flag & SOR_RESULT_LANE) {
			LaneResult result;
			_ipmm->GetResultLane(&result);
		}
		switch (result_flag & SOR_RESULT_MASK_STEREO) {
		case SOR_RESULT_HIST:
			{
				StereoResultHist result;
				_ipmm->GetResultStereoHist(&result);
			}
			break;
		case SOR_RESULT_HOUGH:
			{
				StereoResultHough result;
				_ipmm->GetResultStereoHough(&result);
			}
			break;
		case SOR_RESULT_LABELING:
			{
				StereoResultLabeling result;
				_ipmm->GetResultStereoLabeling(&result);
			}
			break;
		}
	}
#endif
	if (_ipmm->CollectImage()) {
		ulong length = _ipmm->ImageLength();
		pthread_mutex_lock(&_mutex);
		memcpy(_data, _ipmm->ImageData(), length);
		pthread_mutex_unlock(&_mutex);
	} else {
		//usleep(33333);
		//_ipmm->Wait();
	}

	return true;
}

bool SimpleIpmReader::ReadData(uchar *data, int buff_size)
{
	if (buff_size < _length) {
		return false;
	}
	pthread_mutex_lock(&_mutex);
	memcpy(data, _data, _length);
	pthread_mutex_unlock(&_mutex);
	return true;
}



