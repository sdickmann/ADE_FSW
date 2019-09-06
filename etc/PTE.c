//headers
#include <math.h>
#include <polysat/polysat.h>
//#include <libpolydrivers.h>

//definitions
#define MAX_PASS 3000
#define T_STEP 1 //nominal time step
#define PTE_CYCLE 1500
#define SIZE 4 //(n+1) order of temp correction
#define WINDOW 120 //size of window filter
#define CALIBRATION_PERIOD 150 //size of calibration period, seconds
//structures
struct IMUData {
	double t[MAX_PASS];
	float x[MAX_PASS];
	float y[MAX_PASS];
	float z[MAX_PASS];
};

//functions
void minor(long double mat[][SIZE], long double cofac[][SIZE], int r, int c, int n){

	long double mat_det=0;
	int i;
	int j;
	int k=0;
	long double hold_val;

	for (i=0;i<n;i++){
		if (i != r){
			for (j=0;j<n;j++){
				if (j != c){
					hold_val = mat[i][j];
					if (i < r & j < c)
						cofac[i][j] = hold_val;
					else if (i > r & j < c)
						cofac[i-1][j] = hold_val;
					else if (i < r & j > c)
						cofac[i][j-1] = hold_val;
					else if (i > r & j > c)
						cofac[i-1][j-1] = hold_val;
				}
			}
		}
	}
	return;
}

long double det(long double mat[][SIZE], int n){

	long double mat_det=0;
	long double cofac[SIZE][SIZE];
	int i;
	int j;
	int k;
	int m=0;
	long double mat_hold[SIZE][SIZE];
	long double det_hold;
	long double mat_val;

	for (k=0;k<SIZE;k++){
			for (j=0;j<SIZE;j++){
				mat_hold[k][j] = mat[k][j];
			}
	}
	

	if (n != SIZE){
		for (k=0;k<n;k++){
			for (j=0;j<n;j++){
				if (k == 0 & j < n){
					mat[k][j+m] = mat_hold[k][j];
				} else if (k == 0 & j == n){
					mat[k+1][0] = mat_hold[k][j];
					m++;
				} else if (k > 0 & j < n-m){
					mat[k][j+m] = mat_hold[k][j];
				} else if (k>0 & j >= n-m & j <SIZE){
					mat[k+1][m-1] = mat_hold[k][j];
					m++;
				}
			}
		}
	}

	if (n == 1){
		mat_det = mat[0][0];
	} else if (n == 2){
		mat_det = mat[0][0]*mat[1][1] - mat[0][1]*mat[1][0];
	} else {
		for (i = 0; i < n; i++){
			mat_val = mat[0][i];
			minor(mat,cofac,0,i,n);
			det_hold = det(cofac,n-1);
			mat_det += mat_val*pow(-1,i)*det_hold;
		}
	}
	return mat_det;	
}

void inv(long double mat[][SIZE], long double mat_inv[][SIZE], int n){

	int i;
	int j;
	long double cofac[SIZE][SIZE];
	long double mat_det;
	long double minor_mat[SIZE][SIZE];
	long double cofac_mat[SIZE][SIZE];

	for (i = 0; i<n; i++){
		for (j=0;j<n;j++){
			minor(mat, minor_mat, i, j, n);
			cofac[i][j] = pow(-1,i+j+2)*det(minor_mat, n-1);
			cofac_mat[i][j] = cofac[i][j];
		}
	}
	mat_det = det(mat, n);

	for (i=0;i<n;i++){
		for (j=0;j<n;j++){
			mat_inv[i][j] = cofac_mat[j][i]/mat_det;
		}
	}
	
	return;
}

void temp_correction(double time[], double accel[], double temp[], int n, double filtered[]){
	long double X[2*CALIBRATION_PERIOD][SIZE];
	long double XT[SIZE][2*CALIBRATION_PERIOD];
	long double accel_calibration[2*CALIBRATION_PERIOD];
	int i;
	int j;
	long double coef[SIZE]={0};
	long double xtx[SIZE][SIZE]={0};
	int k;
	long double xtx_inv[SIZE][SIZE];
	long double inv_xt[SIZE][2*CALIBRATION_PERIOD]={0};

	// note: this filter was designed for a cubic fit

	for (i=0;i<CALIBRATION_PERIOD;i++){
		X[i][0] = pow(temp[i], 3);
		X[i][1] = pow(temp[i], 2);
		X[i][2] = temp[i];
		X[i][3] = 1;
		X[2*CALIBRATION_PERIOD-i][0] = pow[temp[n-i], 3];
		X[2*CALIBRATION_PERIOD-i][1] = pow(temp[n-i], 2);
		X[2*CALIBRATION_PERIOD-i][2] = temp[n-i];
		X[2*CALIBRATION_PERIOD-i][3] = 1;
		accel_calibration[i] = accel[i];
		accel_calibration[2*CALIBRATION_PERIOD-i] = accel[n-i];
	}
	
	for (i=0; i<2*CALIBRATION_PERIOD; i++){
		for (j=0;j<SIZE;j++){
			XT[j][i] = X[i][j];
		}
	}

	for (i=0;i<SIZE;i++){
		for (j=0;j<SIZE;j++){
			for (k=0;k<2*CALIBRATION_PERIOD;k++){
				xtx[i][j] += XT[i][k]*X[k][j];
			}
		}
	}

	inv(xtx, xtx_inv, SIZE);

	for (i=0;i<SIZE;i++){
		for(j=0;j<2*CALIBRATION_PERIOD;j++){
			for(k=0;k<SIZE;k++){
				inv_xt[i][j] += xtx_inv[i][k]*XT[k][j];
			}
		}
	}

	for (i=0; i<SIZE; i++){
		for (k=0;k<2*CALIBRATION_PERIOD;k++){
			coef[i] += inv_xt[i][k]*accel_calibration[k];
		}
	}

	for (i=0; i<n; i++){
		filtered[i] = accel[i] - (coef[0]*pow(temp[i],3) + coef[1]*pow(temp[i],2) + coef[2]*temp[i] + coef[3]*1);
	}
	return;
}

double mean_window(double data[], int n){
	double sum=0;
	int i;
	for (i=0;i<n;i++){
		sum += data[i];
	}

	return sum/n;
}

double window_filter(double accel[], int window, int n, double filter[]){
	int k;
	double mean_accel[WINDOW];
	int i;
	double th;
	double mean_filter = 0;

	for (k=floor(WINDOW/2); k<=n-floor(WINDOW/2); k++){
		for (i=0; i<WINDOW; i++){
			mean_accel[i]=accel[(int) (k-floor(WINDOW/2)) + i];
		}
		filter[k] = mean_window(mean_accel, WINDOW);
		printf("accel: %f mean: %f\n", accel[k], mean_window(mean_accel,WINDOW));
	}

	for (i=0;i<WINDOW/2;i++){
		filter[i]=filter[WINDOW/2];
		filter[n-i]=filter[n-WINDOW/2];
	}

	for (i=0;i<300;i++){
		mean_filter += filter[i] + filter[n-300];
	}

	mean_filter /= 600;

	for (i=0;i<n;i++){
		filter[i] -= mean_filter;
	}
	
	for (i=0;i<n;i++){
		if (i==0){
			th = filter[i];
		} else if (filter[i] > th){
			th = filter[i];
		}
	}

	th = th/4;

	return th;
}

void PTE_start(int socket, unsigned char cmd, void * data, size_t dataLen, struct sockaddr_in * src)
{
	//this function is used to start PTE and IMU? from ground command

	PROC_cmd_sockaddr(proc, CMD_STATUS_RESPONSE, &status, sizeof(status), src);
}

int sigint_handler(int signum, void *arg)
{
	EVT_exit_loop(PROC_evt(arg));
	return EVENT_KEEP;
}

double PTE_process(struct IMUData data, double t_step, double a_filter[], int array_size, double t[])
{
	double a_mag[(int) floor(PTE_CYCLE/t_step)]; // should hold 20 minutes of data
	double corrected[(int) floor(PTE_CYCLE/t_step)];
	double th; // threshold value
	int i;
	int j; // start of 20 mins of data

	j = floor(5*60/t_step);
	// read IMU
	for (i = 0; i < (int) floor(PTE_CYCLE/t_step); i++){
		a_mag[i] = sqrt(pow(data.x[i],2) + pow(data.y[i],2) + pow(data.z[i],2));
	}

	// bias correction
	temp_correction(data.t, a_mag, data.temp, array_size, corrected);

	// filter
	th = window_filter(corrected, array_size, a_filter);

	for (i=0;i<PTE_CYCLE;i++){
		t[i] = data.t[i+j];
	}

	return th;
}


double PTE(double a_m[], double t[],  double th, double t_step, double *tp_err, int *pass, double *period, double *tp_cent, double tp_prev, double tp_est_prev, double r_p, int array_size)
{
	double mu = 398600.44; // gravitational parameter (km^3/s^2)
	int test = 0; // counter for crossing threshold
	int mark = 0; // current state (above or below threshold)
	int i_1 = 0; // drag pass start marker
	int i_2 = 0; // drag pass end marker
	int n_thresh = 3; // n consecutive values above threshold
	int j;
	int i;
	double dv_m[MAX_PASS]; // trapezoidal integration
	double dv_t[MAX_PASS]; // moment
	double tp2; // drag pass centroid (MET)
	double dV; // delta V (km/s)
	double period_prev; // previous period
	double a1; // semi major axis before pass (km)
	double Vp1; // periapsis velocity before pass (km/s)
	double Vp2; // periapsis velocity after pass
	double a2; // semi major axis after pass (km)
	double tp_est; // periapsis time estimation
	double sum_dv_t;
	double sum_dv_m;
	
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
		dv_m[i] = t_step*(a_m[i]+a_m[i+1])/2;
		dv_t[i] = dv_m[i]*(t[i]+t[i+1])/2;
		sum_dv_m += dv_m[i];
		sum_dv_t += dv_t[i];
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

void PTE_control(struct IMUData data)
{
	double t_step; // time step of data
	double a_m[MAX_PASS]; // filtered acceleration magnitude data
	int array_size; // a_m array size (depends on time step)
	double th; // this should be a pointer
	double tp_est; // periapsis time estimations
	double tp_err_act;
	double *tp_err; // periapsis time estimation error
	int *pass; // pass number pointer
	int pass_act;
	double *period; // orbit period
	double period_act;
	double tp_hist[MAX_PASS]; // periapsis time history (need to make variable)
	double *tp_cent; // centroided periapsis
	double tp_cent_act;
	double tp_prev; // hold previous periapsis
	double tp_est_prev; // hold previous periapsis estimation
	double tp_est_hist[MAX_PASS]; // periapsis estimation time history (need to make variable)
	double r_p; // radius of periapsis
	double t[MAX_PASS];

	// initialize pointers
	pass = &pass_act;	
	*pass = 1; // on initialization
	tp_err = &tp_err_act;
	period = &period_act;
	tp_cent = &tp_cent_act;
	
	// read IMU
	// get radius of periapsis
	r_p = 6563.1; // hardcoded for now
	
	t_step = data.t[1]-data.t[0];
	array_size = floor(PTE_CYCLE/t_step);
	
	th = PTE_process(data, t_step,  a_m, array_size, t); // process raw data, output th, change a_m
	
	tp_est = PTE(a_m, t, th, t_step, tp_err, pass, period, tp_cent, tp_prev, tp_est_prev, r_p, array_size);

	tp_prev = *tp_cent;
	tp_hist[*pass-1] = tp_prev;
	tp_est_prev = tp_est;
	tp_est_hist[*pass-1] = tp_est_prev;

	//schedule next PTE run	
}

int main(int argc, char *argv[])
{
	//ProcessData proc

	//initialize process
	//proc = PROC_init("PTE", WD_DISABLE); //watchdog or no?
	//PROC_signal(proc, SIGINT, &sigint_handler, proc);

	//initialize IMU

	//EVT_start_loop(PROC_event(proc));	

	//cleanup
	//PROC_cleanup(proc);
	return 0;
}
