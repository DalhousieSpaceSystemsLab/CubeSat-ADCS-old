/*
 * Authors: Cathy Song
 * Project: Dalhousie CubeSat
 * SubSystem: ADCS
 * Date:   2019-11-22
 * References: MATLAB code by Dr. Bauer

 * Description: This file is part of the IGRF algorithm used in CubeSat ADCS team.
 *
 */

//#include "pch.h"
#include "igrf.h"
#include <fstream>

using namespace std;

/* FUNCTION DECLARATIONS */
int Parse_IGRF(void);
uint8_t IGRF(double lat_geodetic, float phi, float H, uint16_t year, uint8_t month, uint8_t day);
double calc_factorial(int x);
void calc_sum_legendre(double P[][MAX_MN_VALUE], double dP[][MAX_MN_VALUE], float costheta, float sintheta);

/* GLOBALS */
/* holds the day index for each first day of the month */
uint16_t MonthDayIndex[12] = { 0,31,59,90,120,151,181,212,243,273,304,334 };
/* change in time with respeect to refrence year (based on IGRF txt file) */
float delta_t = 0;

/* storage arrays for IGRF.txt parsing */
double g_nominal[MAX_MN_VALUE][MAX_MN_VALUE];	// IGRF g coefficient nanoTesla (nT)
double h_nominal[MAX_MN_VALUE][MAX_MN_VALUE];	// IGRF h coefficient nanoTesla (nT)
double SV_g[MAX_MN_VALUE][MAX_MN_VALUE];		// secular variation of g (nT/year)
double SV_h[MAX_MN_VALUE][MAX_MN_VALUE];		// secular variation of h (nT/year)

int main()
{	
	int n = 0, m = 0;
	
	/* date variables */
	unsigned int year = 0;
	unsigned int month = 0;
	unsigned int day = 0;
	unsigned int year_reference;
	int leap_year;
	int leap_year_incl;
	int days_in_year;
	float year_current;

	/* RECIEVE DATE AS INPUT */
#ifdef debugigrf
	cout << "Input refrence year: ";
	cin >> year_reference;
	cout << "Input current time (between 2015 01 00 and 2020 12 31) as YYYY MM DD: ";
	cin >> year >> month >> day;
#endif

	/* DETERMINE CURRENT TIME USED FOR CALCULATIONS */

	/* the year can be evenly divided by 4 AND cannot be evenly divided by 100 */
	leap_year = ((year % 400 == 0) || (year % 4 == 0) && (year % 100));
	/* need to include the leap_year in the calculation only if the current date is into March */
	leap_year_incl = (month > 2) ? leap_year : 0;

	/* calculate number of days so far in the current year */
	days_in_year = (MonthDayIndex[month-1] - 1) + day + leap_year_incl;
	year_current = year + (float(days_in_year) / (365 + leap_year));
	delta_t = year_current - year_reference;


	/* parse IGRF data file */
	if (!Parse_IGRF()) {
#ifdef debugigrf
		cout << "unable to open the IGRF file\n";	// diagnostic
#endif
	}


	IGRF(30.166923849507349, 23, 300, year, month, day);
}

uint8_t IGRF(double lat_geodetic, float phi, float H, uint16_t year, uint8_t month, uint8_t day) {
	/************************************************************************
	Description: This function calculates the IGRF algorithm, producing
	B(X, Y, Z)
	
	Author: written by Cathy, originally developed by Dr.Bauer in MATLAB

	Original can be found at the Dal Cubesat Sharepoint: 
		10.ADCS > 08.Testing > IGRF12

	Inputs:
		- lat_geodetic: geodetic latitude (deg) of desired location 
		  (geodetic coordinate) values range between -90 and +90 deg 
		  (but not -90 or +90), phi, year, month, day

		- phi: longitude (deg) of desired location Note: longitude is the 
		  same for both geodetic and geocentric spherical coordinates values 
		  range between -180 and +180 deg

		- H: altitude above Earth's surface (km), which is perpendicular to 
		  the surface of Earth (geodetic coordinate) values range between 
		  -1km to 600km

		- year, month, day: must be between the valid dates corresponding to 
		  the IGRF constants being used. For example: using IGRF12 dates must 
		  be between 2015 01 01 and 2019 12 31

	output: SUCCESS|FAIL diagnostic

	****** NOTE: DIFFERENCES BETWEEN THIS VERSION AND MATLAB VERSION **********

	- g and h values with respect to delta_t is calculated inside the equation
	  for calculating B sum values rather than before. This is to avoid having
	  a second array to copy values into or reading in the file at every new
	  time instance.

	- the entire array for P and dP is calculated once for each time instance
	  instead for each calculation of Pm_n and dPm_n in order to reduce amount
	  of calculations

	*************************************************************************/
	
	/* INITALIZE VAIRABLES*/

	/* loop variables */
	int n = 0, m = 0;

	/* magnetic field */
	double Btheta = 0, Bphi = 0, Br = 0;
	double Btheta_sum, Bphi_sum, Br_sum;
	double dPm_n = 0, Pm_n = 0;
	string temp;

	/* Cartesian vector variables */
	double N = 0, X = 0, Y = 0, Z = 0;

	/* Spherical vector variables */
	double r = 0, lat_geocentric = 0;

	/* geocentric co-lattitude variables */
	float theta_geocentric = 0, costheta = 0, sintheta = 0;

	/* variables for legendre calculation */
	double P[MAX_MN_VALUE][MAX_MN_VALUE] = { 0 };	// initial Legendre functions used in recursive formulation
	double dP[MAX_MN_VALUE][MAX_MN_VALUE] = { 0 };	// initial partial derivatives of Legendre functions used in recursive formulation
	int delta = 0;

	/* B vector variables (north, east, down) */
	double cd = 0, sd = 0;
	double Bphi_geodetic = 0, Btheta_geodetic = 0, Br_geodetic = 0;
	double Bx = 0, By = 0, Bz = 0;


	/* FROM ELLIPSOIDAL TO CARTESIAN */

	N = a_ref/sqrt(1 - e2 * pow(sin(DegToRad(lat_geodetic)), 2));
	X = (N + H)*(cos(DegToRad(lat_geodetic))*cos(DegToRad(phi)));
	Y = (N + H)*(cos(DegToRad(lat_geodetic))*sin(DegToRad(phi)));
	Z = (N*(1 - e2) + H)*sin(DegToRad(lat_geodetic));


	/* FROM CARTESIAN TO SPHERICAL */

	r = sqrt(pow(X, 2.0) + pow(Y, 2.0) + pow(Z, 2.0));
	lat_geocentric = RadToDeg(asin(Z / r));


	/* FROM GEOCENTRIC LATITUDE TO GEOCENTRIC CO-LATITUDE */

	theta_geocentric = 90 - lat_geocentric; // geocentric co-latitude
	/* need sin and cos of geocentric co-latitude for calculating Schmidt 
	   semi/quasi-normalized Legendre polynomial function later */
	costheta = cos(DegToRad(theta_geocentric));
	sintheta = sin(DegToRad(theta_geocentric));

	/* calculate all sums for P and dP */
	calc_sum_legendre(P, dP, costheta, sintheta);

	/* CALCULATE MAGNETIC FIELD STRENGTH VECTOR B EXPRESSED IN 
	GEOCENTRIC SPHERICAL COORDINATES*/
	
	/* loop to calculate nested sum */
	for (n = 1; n < MAX_MN_VALUE; n++) {
		cout << "n = " << n << '\n';
		Btheta_sum = 0, Bphi_sum = 0, Br_sum = 0;
		for (m = 0; m <= n; m++) {
			// does m_des == 0? if it does delta = 1, if not delta = 0
			delta = (m == 0) ? 1 : 0;

			/* calculate schmidt semi-normalized legendre function and partial derivative from desired P and dP */
			Pm_n = sqrt((2 - delta)*(calc_factorial(n - m)) / (calc_factorial(n + m))) * P[n][m];
			dPm_n = sqrt((2 - delta)*(calc_factorial(n - m)) / (calc_factorial(n + m))) * dP[n][m];

			/* calculate magnetic fields from equations from Ref A */
			Btheta_sum = Btheta_sum - dPm_n * ((g_nominal[n][m] + delta_t * SV_g[n][m]) * cos(DegToRad(m*phi)) + (h_nominal[n][m] + delta_t * SV_h[n][m]) * sin(DegToRad(m*phi)));
			Bphi_sum = Bphi_sum - m * Pm_n * (-(g_nominal[n][m] + delta_t * SV_g[n][m]) * sin(DegToRad(m*phi)) + (h_nominal[n][m] + delta_t * SV_h[n][m]) * cos(DegToRad(m*phi)));
			Br_sum = Br_sum + Pm_n * ((g_nominal[n][m] + delta_t * SV_g[n][m]) * cos(DegToRad(m*phi)) + (h_nominal[n][m] + delta_t * SV_h[n][m])*sin(DegToRad(m*phi)));
		}
		Btheta = Btheta + pow(a/r, (n+2)) * Btheta_sum;	//Eq 3b Ref A
		Bphi = Bphi + (1 / sintheta) * pow(a/r, (n+2)) * Bphi_sum; //Eq 3c Ref A
		Br = Br + pow(a/r, (n+2)) * (n+1)*Br_sum; // Eq 3a Ref A
	}

	/* Convert back to geodetic coordinates based on Ref D p782 Eq H-13:
		lat_geocentric = 90-theta_geocentric where lat_geocentric is delta
		and theta_geocentric is the geocentric co-latitude which is 'theta' 
		in Ref D p782 */
	cd = cos(DegToRad((lat_geodetic - lat_geocentric)));
	sd = sin(DegToRad((lat_geodetic - lat_geocentric)));

	Bphi_geodetic = Bphi;
	Btheta_geodetic = Btheta * cd + Br * sd;
	Br_geodetic = Br * cd - Btheta * sd;

	/* Convert to NED: (North, East, Down) (Bx,By,Bz) */
	Bx = -Btheta_geodetic;	// North is opposite direction to co - latitude theta
	By = Bphi_geodetic;		// phi_geodetic is already East
	Bz = -Br_geodetic;		// Down is opposite direction to radial direction r
	
#ifdef debugigrf	// print results
	cout << '\n' << "B (North, East, Down):" << '\n';
 	cout << "Bx = " << Bx << '\n';
	cout << "By = " << By << '\n';
	cout << "Bz = " << Bz << '\n';
#endif

	return 0;
}


int Parse_IGRF(void) {
	/************************************************************************
	Description: This function parses the IGRF.txt file which contains the
	gmn, hmn, and SVgmn, SVhmn values required for the IGRF algorithm.

	Author: Cathy

	input: none
	output: SUCCESS|FAIL diagnostic
	*************************************************************************/
	ifstream IGRFconstants;
	IGRFconstants.open("IGRF12.txt", ifstream::in);

	/* parsing variables */
	int m = 0, n = 0, i = 0;
	string tempdata;	// used to read and toss unneccesary values in the IGRF12 file
#ifdef debugparse
	string userinput;	// used to receive user input to keep terminal open
#endif

	if (!IGRFconstants) {
#ifdef debugparse
		cout << '\n' << "failed to open the file" << '\n';
#endif
		IGRFconstants.close();		// close the file if opening failed
		return FAIL;
	}
	else {
#ifdef debugparse
		cout << "opened the IGRF file\n\n";	// file opening was successful
		cout << "n" << "\t" << "m" << '\t' << "g_nom" << '\t' <<
			"h_nom" << '\t' << "SV_g" << '\t' << "SV_h" << '\n';
#endif
		while (IGRFconstants) {				// read in values
			IGRFconstants >> n >> m;
			IGRFconstants >> g_nominal[n][m];
			IGRFconstants >> h_nominal[n][m];
			IGRFconstants >> SV_g[n][m];
			IGRFconstants >> SV_h[n][m];
			IGRFconstants >> tempdata >> i;
#ifdef debugparse
			cout << n << "\t" << m << "\t" << g_nominal[n][m] << "\t" << h_nominal[n][m] << "\t" << SV_g[n][m] << "\t" << SV_h[n][m] << '\n';
#endif
		}


#ifdef debugparse
		/* print the g values with respect to m and n*/
		cout << "n (x axis), m (y axis):\n";
		cout << "g values(m,n): \n";
		for (n = 0; n < MAX_MN_VALUE; n++) {
			for (m = 0; m < MAX_MN_VALUE; m++) {
				cout << g_nominal[n][m] << "\t";
			}
			cout << "\n";
		}
		/* print the h values with respect to m and n*/
		cout << "\nh values(m,n): \n";
		for (n = 0; n < MAX_MN_VALUE; n++) {
			for (m = 0; m < MAX_MN_VALUE; m++) {
				cout << h_nominal[n][m] << "\t";
			}
			cout << "\n";
		}
		/* print the SVg values with respect to m and n*/
		cout << "SVg values(m,n): \n";
		for (n = 0; n < MAX_MN_VALUE; n++) {
			for (m = 0; m < MAX_MN_VALUE; m++) {
				cout << SV_g[n][m] << "\t";
			}
			cout << "\n";
		}
		/* print the SVh values with respect to m and n*/
		cout << "SVh values(m,n): \n";
		for (n = 0; n < MAX_MN_VALUE; n++) {
			for (m = 0; m < MAX_MN_VALUE; m++) {
				cout << SV_h[n][m] << "\t";
			}
			cout << "\n";
		}
#endif
	}
	IGRFconstants.close();	// close the file
	return SUCCESS;
}

void calc_sum_legendre(double P[][MAX_MN_VALUE], double dP[][MAX_MN_VALUE], float costheta, float sintheta) {
	/************************************************************************
	Description: This function calculates the sums for Schmidt semi/quasi-
	normalized Legendre polynomial function of degree n and order m: 
	Pm_n(costheta)along with its partial derivative with respect to costheta: 
	dPm_n(costheta)

	Author: written by Cathy, originally developed by Dr.Bauer in MATLAB

	Original can be found at the Dal Cubesat Sharepoint:
		10.ADCS > ...

	Inputs:
		- empty array for P[][] and dP[][]
		- n: desired degree to calculate
		- m: desired order to calculate 
		- costheta: cosine of ceocentric latitude
		- sintheta: sine of geocentric latitude
	Outputs:
		completed P[][] and dP[][]

	*************************************************************************/

	long double Pm_n = 0, dPm_n = 0;	// temp values for looping calculation
	int n = 0, m = 0;					// temp values for looping calculation
	float delta = 0;					// temp value

	/* define initial Legendre functions used in recursive formulation */
	P[0][0] = 1;
	P[1][0] = costheta;
	P[1][1] = sintheta;

	/* define initial partial derivatives of Legendre functions used in recursive formulation */
	dP[0][0] = 0;
	dP[1][0] = -sintheta;
	dP[1][1] = costheta;
	/* calculate Pm_n and dPm_n using recursive formulation */
	for (n = 2; n <MAX_MN_VALUE; n++) {
		for (m = 0; m<MAX_MN_VALUE; m++){
			if(m==0){
				// Eq 10.93a
				P[n][m] = (1/float(n)) * ( (2*n - 1)*costheta*P[n-1][m] - (n-1)*P[n-2][m] );
				// Eq RJB 1
				dP[n][m] = (1/float(n)) * ( (2*n - 1)*(costheta*dP[n-1][m] - sintheta*P[n-1][m]) - (n-1)*dP[n-2][m] );
			}
			else if (m == n) {
				// Eq 10.93c
				P[n][m] = (2*n - 1)*sintheta*P[n-1][m-1];
				// Eq RJB 3
				dP[n][m] = (2*n - 1)*(sintheta*dP[n-1][m-1] + costheta*P[n-1][m-1]);
			}
			else {	// 0 < m < n
				// Eq 10.93b
				P[n][m] = costheta * P[n - 1][m] + (n + m - 1)*sintheta*P[n - 1][m - 1];
				// Eq RJB 2
				dP[n][m] = costheta * dP[n - 1][m] - sintheta * P[n - 1][m] + (n + m - 1)*(sintheta*dP[n - 1][m - 1] + costheta * P[n - 1][m - 1]);
			}
		}
	}
	
}

double calc_factorial(int x) {
	/************************************************************************
	Description: This function calculates factorial of an input x

	Author: written by Cathy

	Input: x = number to be calculated on
	Output: result = x!

	*************************************************************************/
	long double result = 1;
	int i = 0;

	if(x!=0) {
		// loop through to obtain x*(x-1)*(x-2)*...*2
		for (i = 2; i <= x; i++) {
			result = result * i;
		}
	}
	return result;
}
