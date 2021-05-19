#include <stdio.h>
#include <math.h>
#include "../Errors.h"
#include "sun_vector_reference.h"

int H, Min, S;
double J0, T0, S_lon_m, M_s, Y, lon_S_ec, epsilon, r_S, r_S_I[3][1], e_ES[3][1];


double sind(double x) {
	return sin(x * M_PI / 180);
}

double cosd(double x) {
	return cos(x * M_PI / 180);
}

ret_val SunReference(uint16_t year, uint16_t month, uint16_t day, double UT, Eigen::Vector3d& sun_reference_unit_vector) {

	/*-------------------------------------------------------------------------
	Description: This function calculates reference sun vector in ECI frame
	based on Matlab code found in 
	DalhousieSpaceSystemsLab/CubeSat-ADCS/Matlab/Sun_Vector_Reference

	Author: written by Mark MacGillivray, edited by Rutwij Makwana, originally 
	developed by Dr.Bauer in MATLAB

	Inputs:
		- year: Current year

		- month: Current month

		- day: Current day

		- UT: Universal time (UTC), decimal hours

		- sun_reference_unit_vector: Output reference vector

	output: SUCCESS|FAIL diagnostic
	-------------------------------------------------------------------------*/

	//Separate time into hours, minutes, and seconds:
	H = (int)UT; // 0 to 24
	Min = (int)((UT - H) * 60);
	S = ((UT - H) * 60 - Min) * 60;

	//Calculate accurate-to-time Julian date, with leap-seconds conceptually
	//ignored (Markley 2.68):
	J0 = 1721013.5 + (367 * year) - (int)((7.0 / 4.0) * (year + (int)((month + 9) / 12))) + (int)(275 * month / 9) + day + (((60.0 * H) + Min + (S / 60.0)) / 1440.0);

	//Time in Julian centuries, w/r/t above Julian date (Markley 11.49)
	T0 = (J0 - 2451545.0) / 36525.0;

	/*From "Fundamentals of Astrodynamics and Applications" by D. Vallado, the
	position of the sun relative to the Earth can be determined "imprecisely"
	(versus measured ephemerides values from JPL etc.) via the following
	J2000 derivation with 0.01 deg accuracy (p. 265).
	(The same equations are given in Fundamentals of Spacecraft Attitude
	Determination and Control pp. 420-422. Equation numbers reference
	both)*/

	//Mean longitude of the sun in degrees w/r/t time in Julian centuries (Markley 11.48a)
	S_lon_m = 280.460 + (36000.771 * T0);

	// Sun mean anomaly, deg, "low-precision" w/r/t " " " (Markley 11.48b)
	M_s = 357.5277233 + (35999.05034 * T0);

	// "Reduce both [...] to the range of 0 deg to 360 deg"
	S_lon_m = fmod(S_lon_m, 360.0); // deg
	M_s = fmod(M_s, 360.0); // deg

	/* Ecliptic longitude via "equation of centre" (Vallado p. 81) where 0.016708617 is
	the "mean eccentricity of Earth's orbit around the sun". Unlike the
	textbook, the equation is solved to the fourth order.
	lon_S_ec = M_s + 2*0.016708617*sind(M_s) + 5/4*0.016708617^2*sind(2*M_s) + ...
		0.016708617^3/12*(13*sind(3*M_s) - 3*sind(M_s)) + 0.016708617^4/96*(103*sind(4*M_s) - 44*sind(2*M_s));*/

		// Or, use the simplified formula (Markley 11.50)
	lon_S_ec = S_lon_m + (1.914666471 * sind(M_s)) + (0.019994643 * sind(2.0 * M_s)); // deg

	// ****** Assuming the ecliptic latitude of the sun is 0 deg ******

	// Obliquity of the ecliptic is approximated as (Markley 11.51):
	epsilon = 23.439291 - (0.0130042 * T0); // deg

	// Magnitude of Earth's positon w/r/t sun, in astronomical units (AU) (Markley 11.53)
	r_S = 1.000140612 - (0.016708617 * cosd(M_s)) - (0.000139589 * cosd(2.0 * M_s)); // AU

	// Vector in the J2000 ECI frame (assuming "mean equinox" is synonymous with
	// vernal equinox) (Markley 11.52)

	r_S_I[0][0] = r_S * cosd(lon_S_ec); // AU
	r_S_I[1][0] = r_S * cosd(epsilon) * sind(lon_S_ec); // AU
	r_S_I[2][0] = r_S * sind(epsilon) * sind(lon_S_ec); // AU


	 // Obtain unit vector:
	 for (int i = 0; i < 3; i++) {
	 	e_ES[i][0] = r_S_I[i][0] / r_S; 
	 }

	 sun_reference_unit_vector << e_ES[0][0], e_ES[1][0], e_ES[2][0];

	return SUCCESS;
}
