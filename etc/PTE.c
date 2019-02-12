//headers
#include <polysat/polysat.h>
//include IMU driver

//definitions

//structures

//functions
void PTE_status(int socket, unsigned char cmd, void * data, size_t dataLen, struct sockaddr_in * src)
{
	PROC_cmd_sockaddr(PTE->proc, CMD_STATUS,RESPONSE; &status, sizeof(status), src);
	//why PTE->proc
}

int main(int argc, char *argv[])
{
	//initialize process
	PTE.proc
	//why PTE.proc

	//filter data

	//run PTE

	//set next IMU run time

	//package status

	//cleanup
	return 0;
}
