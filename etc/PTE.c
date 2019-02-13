//headerss
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
	"IMU", DEVICE_LOCATION_MB, accel, 0, NULL, 0, NULL, NULL, 0 }; //DEVICE_LOCATION_MB just a filler

struct IMUData {
	int32_t x[], y[], z[];
}

//functions
static void marshal_accel(struct SensorInfo *si, void *dst)
{
	struct AccelerometerSensor *accel = (struct AccelerometerSensor*)si->sensor;
	struct IMUData *ad = (struct IMUData*)dst;
	AccelData data;

	if (accel->read) {
		accel->read(accel, &data);
		ad->x = hton1(data.x_result);
		ad->y = hton1(data.y_result);
		ad->z = hton1(data.z_result);
	}
} //note that this is also a placeholder based off ADCS example and needs to be customized

void PTE_run(int socket, unsigned char cmd, void * data, size_t dataLen, struct sockaddr_in * src)
{
	//actual PTE goes here
	//filter then PTE

	PROC_cmd_sockaddr(proc, CMD_STATUS_RESPONSE, &status, sizeof(status), src);
}

int sigint_handler(int signum, void *arg)
{
	EVT_exit_loop(PROC_evt(arg));
	return EVENT_KEEP;
}

int main(int argc, char *argv[])
{
	//initialize process
	proc = PROC_init("PTE", WD_DISABLE); //watchdog or no?
	PROC_signal(proc, SIGINT, &sigint_handler, proc);

	//initialize IMU

	EVT_start_loop(PROC_event(proc));	
	//set next IMU run time

	//cleanup
	PROC_cleanup(proc);
	return 0;
}
