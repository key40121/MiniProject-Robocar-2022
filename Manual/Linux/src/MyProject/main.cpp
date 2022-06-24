#include <unistd.h>
#include "RcControl.h"

using namespace zmp::zrc;

int main()
{
	RcControl rc;
	rc.init();
	rc.SetServoEnable(1);
	rc.SetSteerAngle(30);
    sleep(1);
	rc.SetSteerAngle(0);
	sleep(1);
	rc.SetSteerAngle(-30);
	sleep(1);
	rc.SetSteerAngle(0);
	sleep(1);
	rc.SetServoEnable(0);
}
