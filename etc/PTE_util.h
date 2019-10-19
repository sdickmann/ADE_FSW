//definitions
#define MAX_PASS 3000
#define T_STEP 1 //nominal time step
#define PTE_CYCLE 1500
#define SIZE 4 //(n+1) order of temp correction
#define WINDOW 120 //size of window filter
#define CALIBRATION_PERIOD 150 //size of calibration period, seconds
#define SAFE_MODE 0
#define ACTIVE_MODE 1

//structures
struct IMUData {
	double t[MAX_PASS];
	double x[MAX_PASS];
	double y[MAX_PASS];
	double z[MAX_PASS];
	double temp[MAX_PASS];
};

//functions
void minor(long double mat[][SIZE], long double cofac[][SIZE], int r, int c, int n);
long double det(long double mat[][SIZE], int n);
int inv(long double mat[][SIZE], long double mat_inv[][SIZE], int n);
void temp_correction(double time[], double accel[], double temp[], int n, double filtered[]);
double mean_window(double data[], int n);
double window_filter(double accel[], int n, double filter[]);