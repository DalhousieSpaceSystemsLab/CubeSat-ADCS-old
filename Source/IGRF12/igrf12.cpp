//This file is the main file that runs the IGRF12 module
#include "igrf12.hpp"

#include "pch.h"
#include "igrf12.h"

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

#ifdef debugigrf
	cout << "N = " << N << '\n';
 	cout << "X = " << X << '\n';
	cout << "Y = " << Y << '\n';
	cout << "Z = " << Z << '\n';
#endif

	return 0;
}