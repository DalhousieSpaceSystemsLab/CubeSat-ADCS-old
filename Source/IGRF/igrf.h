#pragma once
#ifndef IGRF12_H
#define IGRF12_H

#include <cmath>
#include <iostream>
#include <string>
#include <stdint.h>

/* OPTIONS FOR DEBUGGING IGRF ALGORITHM AND PARSER */
#define debugigrf		// outputs to terminal for testing igrf	
#define	debugparse		// outputs to terminal for testing the parse function

/* constants */
enum returnvals { FAIL, SUCCESS };
#define IGRF_READLINES	104						// number of lines to read for IGRF.txt
#define MAX_MN_VALUE	13+1					// max m and n values, (ignoring 0)

constexpr auto a_ref = 6378.137;	// km, semimajor axis of reference model ellipsoide;
constexpr auto a = 6371.2;			// km, magnetic spherical reference radius (Earth radius)
constexpr auto f = 1/298.257223563;	// wgs84Ellipsoid reference ellipsoid for Earth flattening;
const double e2 = 2*f-pow(f,2);       // eccentricity^2, Ref C, p6

/* used for converting from radians to degrees */
#define PI 3.14159265
#define DegToRad(X)	X*PI/180
#define RadToDeg(X)	X*180/PI


#endif