//headers
#include <polysat/polysat.h>
//include IMU driver

//definitions

//structures
struct SensorInfo {
	const char *name;
	const char *location;
	const char *type;
	int flags;
	void (*marshal)(struct SensorInfo *sensor, void *dst);
	int offset;
	struct Sensor *sensor;
	struct DeviceInfo *dev_info;
	int disabled;
}

struct SensorInfo IMU[] = {
	"IMU", DEVICE_LOCATION_MB, accel, 0, NULL, 0, NULL, NULL, 0 };

//functions
void PTE_status(int socket, unsigned char cmd, void * data, size_t dataLen, struct sockaddr_in * src)
{
	PROC_cmd_sockaddr(proc, CMD_STATUS_RESPONSE; &status, sizeof(status), src);
}

int sigint_handler(int signum, void *arg)
{
	EVT_exit_loop(PROC_evt(arg));
	return EVENT_KEEP;
}

int main(int argc, char *argv[])
{
	//initialize process
	proc = PROC_init("PTE");
	PROC_signal(proc, SIGINT, &sigint_handler, proc);

	//initialize IMU

	//filter data

	//run PTE

	//set next IMU run time

	//package status

	//cleanup
	return 0;
}
