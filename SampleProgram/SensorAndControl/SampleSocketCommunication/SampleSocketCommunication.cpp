#include <stdlib.h>
#include <iostream>
#include <string.h>
#include <signal.h>

using namespace std;

#include "RczCommServer.h"
#include "ServerApp.h"

void funcx(int sig);
static ServerApp *zs; 
int main(int argc, char *argv[]) {
	signal(SIGINT, funcx);
	zs = new ServerApp(9292);
	zs->Init();
	zs->Start();

	while(1) {
		zs->Process();
		usleep(10 * 1000);
	}
	delete zs;
	return 0;
}

void funcx(int sig) {
    zs->Stop();
	signal(SIGINT, SIG_DFL);
	raise(SIGINT);
}

