/*
 * .h
 *
 *  Created on: 2009/04/09
 *      Author: segawa
 */

#ifndef __SIMPLE_IPM_READER_H__
#define	__SIMPLE_IPM_READER_H__

#include "IpmManager.h"
#include <pthread.h>

using namespace zmp::zrc;


#define SIMPLE_IMAGE_SIZE (320*240)

class SimpleIpmReader 
{
public:
	SimpleIpmReader();
	virtual ~SimpleIpmReader();

public:
	bool Init();
	bool Start();
	bool ReadData(uchar *data, int buff_size);
	static void *ThreadEntry(void *arg);
	void Thread();

protected:
	bool collect_proc();

private:
	IpmManager *_ipmm;
	pthread_mutex_t _mutex;
	int _length;
	uchar *_data;
	pthread_t _th;

};


#endif


