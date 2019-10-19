#include <PTE_util.h>

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

int inv(long double mat[][SIZE], long double mat_inv[][SIZE], int n){

	int i;
	int j;
	long double cofac[SIZE][SIZE];
	long double mat_det;
	long double minor_mat[SIZE][SIZE];
	long double cofac_mat[SIZE][SIZE];
	int is_invertible;
	
	for (i = 0; i<n; i++){
		for (j=0;j<n;j++){
			minor(mat, minor_mat, i, j, n);
			cofac[i][j] = pow(-1,i+j+2)*det(minor_mat, n-1);
			cofac_mat[i][j] = cofac[i][j];
		}
	}
	
	mat_det = det(mat, n);

	if (mat_det) {
		is_invertible = 1;
		for (i=0;i<n;i++){
			for (j=0;j<n;j++){
				mat_inv[i][j] = cofac_mat[j][i]/mat_det;
			}
		}
	} else {
		is_invertible = 0;
		for (i=0;i<n;i++){
			for (j=0;j<n;j++){
				mat_inv[i][j] = cofac_mat[j][i];
			}
		}
	}
	
	return is_invertible;
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
	int invertible;
	
	// note: this filter was designed for a cubic fit

	for (i=0;i<CALIBRATION_PERIOD;i++){
		X[i][0] = pow(temp[i], 3);
		X[i][1] = pow(temp[i], 2);
		X[i][2] = temp[i];
		X[i][3] = 1;
		X[2*CALIBRATION_PERIOD-i][0] = pow(temp[n-i], 3);
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

	if (inv(xtx, xtx_inv, SIZE)) {
		for (i=0;i<SIZE;i++){
			for(j=0;j<2*CALIBRATION_PERIOD;j++){
				for(k=0;k<SIZE;k++){
					inv_xt[i][j] += xtx_inv[i][k]*XT[k][j];
				}
			}
		}
		
		for (i=0; i<4; i++){
			for (k=0;k<2*CALIBRATION_PERIOD;k++){
				coef[i] += inv_xt[i][k]*accel_calibration[k];
			}
		}

		for (i=0; i<n; i++){
			filtered[i] = accel[i] - (coef[0]*pow(temp[i],3) + coef[1]*pow(temp[i],2) + coef[2]*temp[i] + coef[3]*1);
		}
	} else {
		for (i=0; i<n; i++){
			filtered[i] = accel[i];
		}
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

double window_filter(double accel[], int n, double filter[]){
	int k;
	double mean_accel[WINDOW];
	int i;
	double th;
	double mean_filter = 0;

	for (k=floor(WINDOW/2)-1; k<n-floor(WINDOW/2); k++){
		for (i=0; i<WINDOW; i++){
			mean_accel[i]=accel[(int) (k-floor(WINDOW/2)) + i];
		}
		filter[k-1] = mean_window(mean_accel, WINDOW);
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