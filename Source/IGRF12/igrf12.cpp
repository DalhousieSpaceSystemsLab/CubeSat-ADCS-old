//This file is the main file that runs the IGRF12 module

#include "igrf12.hpp"

using namespace std;

uint16_t MonthDayIndex[12] = { 0,31,59,90,120,151,181,212,243,273,304,334 };

uint8_t IGRF(float lat_geodetic, float phi, float H, uint16_t year, uint8_t month, uint8_t day);

int main()
{	
	/* */
	if (!Parse_IGRF()) {
		cout << "unable to open the IGRF file\n";	// diagnostic
	}

#ifdef debugigrf
	/* receive a date input from the monitor */
	uint16_t year = 0;
	uint8_t month = 0;
	uint8_t day = 0;
	cout << "Input current time (between 2015 01 00 and 2020 12 31) as YYYY MM DD: ";
	cin >> year >> month >> day;
#endif
	IGRF(25, -25, 400, year, month, day);
}

uint8_t IGRF(float lat_geodetic, float phi, float H, uint16_t year, uint8_t month, uint8_t day) {
	/************************************************************************
	Description: This function parses the IGRF.txt file which contains the
	gmn, hmn, and SVgmn, SVhmn values required for the IGRF algorithm.

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
	*************************************************************************/
	
	
	/* the year can be evenly divided by 4 AND cannot be evenly divided by 100 */
	bool leap_year = ((year % 400 == 0) || (year % 4 == 0) && (year % 100));
	/* need to include the leap_year in the calculation only if the current date is into March */
	bool leap_year_incl = (month > 2) ? leap_year : 0;

	/* calculate number of days so far in the current year */
	uint16_t days_in_year = (MonthDayIndex[month] - 1) + day + leap_year_incl;
	uint16_t year_current = year + days_in_year / (365 + leap_year);
	
	double N = a_ref/sqrt(1 - e2 * pow(sin(DegToRad(lat_geodetic)), 2));
	/* from ellipsoidal to Cartesian */
	double X = (N + H)*(cos(DegToRad(lat_geodetic))*cos(DegToRad(phi)));
	double Y = (N + H)*(cos(DegToRad(lat_geodetic))*sin(DegToRad(phi)));
	double Z = (N*(1 - e2) + H)*sin(DegToRad(lat_geodetic));

#ifdef debugigrf	// print results
	cout << '\n' << "Cartesian calculations:" << '\n';
	cout << "N = " << N << '\n';
 	cout << "X = " << X << '\n';
	cout << "Y = " << Y << '\n';
	cout << "Z = " << Z << '\n';
#endif

	/* from Cartesian to spherical */
	double r = sqrt(pow(X, 2) + pow(Y, 2) + pow(Z, 2));
	double lat_geocentric = RadToDeg(asin(Z / r));

#ifdef debugigrf	// print results
	cout << '\n' << "Spherical calculations:" << '\n';
	cout << "r = " << r << '\n';
	cout << "lat_geocentric = " << lat_geocentric << '\n';
#endif
	/* from geocentric latitude to geocentric co-latitude */
	double theta_geocentric = 90 - lat_geocentric; // geocentric co-latitude
	/* need sin and cos of geocentric co-latitude for calculating Schmidt 
	   semi/quasi-normalized Legendre polynomial function later */
	double costheta = cos(DegToRad(theta_geocentric));
	double sintheta = sin(DegToRad(theta_geocentric));
	
#ifdef debugigrf	// print results
	cout << '\n' << "Co-latitude calculations:" << '\n';
	cout << "theta_geocentric = " << theta_geocentric << '\n';
	cout << "costheta = " << costheta << '\n';
	cout << "sintheta = " << sintheta << '\n';
#endif

	/* initialize magnetic field */
	double Btheta = 0, Bphi = 0, Br = 0;
	double Btheta_sum, Bphi_sum, Br_sum;
	double dPm_n = 0, Pm_n = 0;
	/* initialize loop variables */
	uint8_t n = 0, m = 0;
	/* loop to calculate nested sum */
	for (n = 0; n < MAX_MN_VALUE; n++) {
		Btheta_sum = 0, Bphi_sum = 0, Br_sum = 0;
		for (m = 0; m < n; m++) {
			/* calculate Schmidt semi / quasi - normalized Legendre polynomial 
			function of degree n and order m, along with its partial derivative */
			dPm_n = 0; // gotten from legendre();
			Pm_n = 0; // gotten from legendre();

			/* calculate magnetic fields from equations from Ref A */
			Btheta_sum = Btheta_sum - dPm_n * (g_nominal[n][m]*
				cos(DegToRad(m*phi)) + h_nominal[n][m] * sin(DegToRad(m*phi)));
			
			Bphi_sum = Bphi_sum - m*Pm_n * (-g_nominal[n][m] *
				sin(DegToRad(m*phi)) + h_nominal[n][m] * cos(DegToRad(m*phi)));
			
			Br_sum = Br_sum + Pm_n * (g_nominal[n][m] * 
				cos(DegToRad(m*phi)) + h_nominal[n][m] * sin(DegToRad(m*phi)));
		}

		Btheta += pow(a_ref / r, n + 1)*Btheta_sum;	//Eq 3b Ref A

		Bphi += 1 / sintheta * (pow(a_ref / r, n + 1)*Bphi_sum); //Eq 3c Ref A

		Br += pow(a_ref / r, n + 1)*n*Br_sum; // Eq 3a Ref A
	}

	/* Convert back to geodetic coordinates based on Ref D p782 Eq H-13:
		lat_geocentric = 90-theta_geocentric where lat_geocentric is delta
		and theta_geocentric is the geocentric co-latitude which is 'theta' 
		in Ref D p782 */
	double cd = cos(DegToRad(lat_geodetic - lat_geocentric));
	double sd = sin(DegToRad(lat_geodetic - lat_geocentric));
	double Bphi_geodetic = Bphi;
	double Btheta_geodetic = Btheta * cd + Br * sd;
	double Br_geodetic = Br * cd - Btheta * sd;

	/* Convert to NED: (North, East, Down) (Bx,By,Bz) */
	double Bx = -Btheta_geodetic;	// North is opposite direction to co - latitude theta
	double By = Bphi_geodetic;		// phi_geodetic is already East
	double Bz = -Br_geodetic;		// Down is opposite direction to radial direction r
	
	return 0;
}