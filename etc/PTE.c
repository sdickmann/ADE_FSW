//headers
#include <math.h>
#include <polysat/polysat.h>
#include <PTE_func.h>

// global variables
static ProcessData *proc = NULL;
static double r_p; // Radius of periapsis (km)
static int mode; // Flag for mode (0: safe, 1: active)
static int pass; // Pass number	
static long double tp_err_act; // Current periapsis error
static long double period_act; // Current orbit period
static long double tp_cent_act; // Current centroid
static int listen_IMU = 0; // Listen to IMU or not
static long double tp_hist[MAX_PASS] = {0}; // Periapsis time history
static long double tp_est_hist[MAX_PASS] = {0}; // Periapsis estimation time history
static long double tp_err_hist[MAX_PASS] = {0}; // Error time history
static double th; // Threshold value
static double dV; // Delta-V (km/s)

//functions
double PTE_process(struct IMUData data, double t_step, double a_filter[], int array_size, double t[])
{
	double a_mag[(int) floor(PTE_CYCLE/t_step)]; // should hold 25 minutes of data
	double corrected[(int) floor(PTE_CYCLE/t_step)];
	int i;

	// read IMU
	for (i = 0; i < (int) floor(PTE_CYCLE/t_step); i++){
		a_mag[i] = sqrt(pow(data.x[i],2) + pow(data.y[i],2) + pow(data.z[i],2));
	}

	// bias correction
	temp_correction(data.t, a_mag, data.temp, array_size, corrected);

	// filter
	th = window_filter(corrected, array_size, a_filter);

	for (i=0;i<PTE_CYCLE;i++){
		t[i] = data.t[i];
	}

	return th;
}

long double PTE(double a_m[], double t[],  double th, double t_step, long double *tp_err, int *pass, long double *period, long double *tp_cent, long double tp_prev, long double tp_est_prev, double r_p, int array_size)
{
	double mu = 398600.44; // gravitational parameter (km^3/s^2)
	int test = 0; // counter for crossing threshold
	int mark = 0; // current state (above or below threshold)
	int i_1 = 0; // drag pass start marker
	int i_2 = 0; // drag pass end marker
	int n_thresh = 3; // n consecutive values above threshold
	int j;
	int i;
	long double dv_m; // trapezoidal integration
	long double dv_t; // moment
	long double tp2; // drag pass centroid (MET)
	double dV; // delta V (km/s)
	long double period_prev; // previous period
	long double a1; // semi major axis before pass (km)
	double Vp1; // periapsis velocity before pass (km/s)
	double Vp2; // periapsis velocity after pass
	long double a2; // semi major axis after pass (km)
	long double tp_est; // periapsis time estimation
	long double sum_dv_t;
	long double sum_dv_m;
	
	// threshold filter
	for (j = 0; j < array_size; j++){
		if (mark == 0){
			if (a_m[j] > th){
				test=test+1;
				if (test == n_thresh){
					if (i_1 == 0)
						i_1 = j-2;
					mark = 1;
					test = 0;
				}
			}
			else
				test = 0;
			}
		else{
			if (a_m[j] < th){
				test = test + 1;
				if (test == n_thresh){
					i_2 = j-2;
					mark = 0;
					test = 0;
				}
			}
			else
				test = 0;
			}
		}
	
	
	// threshold never reached
	if (i_1 == 0){
		printf("threshold never reached!\n");
		i_2 = 0;
	}

	// still above threshold
	if (mark == 1)
		i_2 = array_size;
	
	// acceleration integration and time centroid
	i = i_1;
	sum_dv_m = 0;
	sum_dv_t = 0;
	while (i<i_2 && a_m[i+1]==a_m[i+1]){
		dv_m = t_step*(a_m[i]+a_m[i+1])/2;
		dv_t = dv_m*(t[i]+t[i+1])/2;
		sum_dv_m += dv_m;
		sum_dv_t += dv_t;
		i++;
	}
	tp2 = sum_dv_t/sum_dv_m;
	*tp_cent = tp2;
	dV = sum_dv_m/1000;
	// initial startup
	if (*pass == 1){
		tp_est = 0;
		*tp_err = 0;
		*pass = *pass + 1;
		*period = 0;
		return tp_est;
	}

	// orbit period
	period_prev = tp2 - tp_prev; // period of last orbit (sec)
	a1 = pow((pow(period_prev/(2*M_PI), 2) * mu), (double) 1/3);
	Vp1 = sqrt(2*(mu/r_p-mu/(2*a1)));
	Vp2 = Vp1 - dV;
	a2 = -mu/(2*(pow(Vp2, 2)/2 - mu/r_p));
	*period = 2*M_PI*sqrt(pow(a2,3)/mu);
	tp_est = tp2 + *period;

	if (*pass == 2){
		*tp_err = 0;
		*pass = *pass + 1;
		return tp_est;
	}
	
	*tp_err = tp_est_prev - tp2;

	*pass = *pass + 1;
	return tp_est;
}

void IMU_trigger(int socket, unsigned char cmd, void *data, size_t dataLen, struct sockaddr_in *fromAddr){
	
	IMUData accel_data;
	
	if (listen_IMU){
		// (!) make sure input data is in correct format
		accel_data = data;
	
		// run PTE using IMU data
		PTE_control(data, mode);
	}
	
	return;
}

void reschedule_IMU(){
	
	cmd = 1; // (!) command number for IMU rescheduling
	
	// (!) Command IMU
	PROC_cmd(proc, cmd, mode, sizeof(mode), "IMU");
	
}
	
void PTE_control(struct IMUData data, int mode)
{
	double t_step; // Time step of data (s)
	double a_m[MAX_PASS]; // Filtered acceleration magnitude data
	int array_size; // a_m array size (depends on time step)
	long double tp_est; // Periapsis time estimations
	long double *tp_err; // Periapsis time estimation error
	int *pass; // Pass number pointer
	long double *period; // Orbit period pointer
	long double *tp_cent; // Centroided periapsis
	long double tp_prev; // Hold previous periapsis
	long double tp_est_prev; // Hold previous periapsis estimation
	double t[MAX_PASS];
	double 

	// Initialize pointers
	pass = &pass_act;	
	tp_err = &tp_err_act;
	period = &period_act;
	tp_cent = &tp_cent_act;
	
	// (!) Read latest data
	tp_prev = tp_hist[*pass-1];
	tp_est_prev = tp_est_hist[*pass-1];
	//r_p = read_r_p();
	
	t_step = data.t[1]-data.t[0];
	array_size = floor(PTE_CYCLE/t_step);
	
	th = PTE_process(data, t_step,  a_m, array_size, t); // process raw data, output th, change a_m
	
	tp_est = PTE(a_m, t, th, t_step, tp_err, pass, period, tp_cent, tp_prev, tp_est_prev, r_p, array_size);

	// (!) Store pass centroids and time estimations so they can be read again
	tp_hist[*pass-1] = tp_cent_act;
	tp_est_hist[*pass-1] = tp_est;
	tp_err_hist[*pass-1] = tp_err_act;
	
	// (!) write to PTT
	//PTT_write(tp_est);
	
	// reschedule IMU based on mode
	reschedule_IMU();
	
}

static int sigint_handler(int signum, void *arg)
{
   EVT_exit_loop(arg);
   return EVENT_KEEP;
}
	
void start(int socket, unsigned char cmd, void *data, size_t dataLen, struct sockaddr_in *fromAddr){
	// (!) read R_p from somewhere
	//if (R_p uplinked)
		//r_p = read_r_p();
	//else
		r_p = 6563.1; // from launch vehicle
	
	mode = SAFE_MODE; // safe mode on start
	listen_IMU = 1; // accept data from IMU flag
	printf("Started\n");
	return;
}

void safe_mode(int socket, unsigned char cmd, void *data, size_t dataLen, struct sockaddr_in *fromAddr){

	mode = SAFE_MODE; // switch mode
	
	return;	
}

void active_mode(int socket, unsigned char cmd, void *data, size_t dataLen, struct sockaddr_in *fromAddr){
	
	mode = ACTIVE_MODE; // switch mode
	
	return;
}

void status(int socket, unsigned char cmd, void *data, size_t dataLen, struct sockaddr_in *fromAddr){
	
	struct PTEStatus {
		int pass;
		double threshold;
		long double delta_V;
		long double error;
		long double estimation;
	};
		
	PTEStatus status;
	
	status.pass = pass;
	status.threshold = th;
	status.delta_V = dV;
	status.error = tp_err_act;
	status.estimation = tp_est_hist[pass-1];
	
	PROC_cmd_sockaddr(proc, CMD_STATUS_RESPONSE, &status, sizeof(status), src);
}

int main(int argc, char *argv[])
{
	//initialize process
	proc = PROC_init("PTE", WD_DISABLED); //watchdog disabled for testing
	
	//SIGINT Handler
	PROC_signal(proc, SIGINT, &sigint_handler, proc);

	//Initialize variables in case IMU starts without starting PTE
	r_p = 6563.1; // from launch vehicle
	mode = SAFE_MODE; // default to safe mode
	
	//Start main event loop (idk what this does)
	EVT_start_loop(PROC_event(proc));	

	//Cleanup on exit
	PROC_cleanup(proc);
	return 0;
}
