#pragma once
#ifndef IGRF12_H
#define IGRF12_H

#include <cmath>
#include <iostream>
#include <string>
#include <stdint.h>
#include "Eigen/Core"
#include "../Errors.h"

/* OPTIONS FOR DEBUGGING IGRF ALGORITHM AND PARSER */
#define debugigrf		// outputs to terminal for testing igrf
//#define	debugparse		// outputs to terminal for testing the parse function

#define IGRF_COEFF_FILE "../../../Source/IGRF/IGRF13.txt"

/* constants */
#define IGRF_READLINES	104						// number of lines to read for IGRF.txt
#define MAX_MN_VALUE	13+1					// max m and n values, (ignoring 0)

constexpr auto a_ref = 6378.137;	// km, semimajor axis of reference model ellipsoide;
constexpr auto a = 6371.2;			// km, magnetic spherical reference radius (Earth radius)
constexpr auto f = 1/298.257223563;	// wgs84Ellipsoid reference ellipsoid for Earth flattening;
const double e2 = 2*f-pow(f,2);       // eccentricity^2, Ref C, p6
const double REF_YEAR = 2020;       // reference year for IGRF model

/* used for converting from radians to degrees */
#define PI 3.141592653589793
#define DegToRad(X)	X*PI/180
#define RadToDeg(X)	X*180/PI

// IGRF function with reference year arg added
// extern uint8_t IGRF_with_ref(double lat_geodetic, double phi, double H, uint16_t year, uint8_t month, uint8_t day, uint16_t ref_year = 2020);
ret_val MagReference(double lat_geodetic, double phi, double H, uint16_t year, uint8_t month, uint8_t day, Eigen::MatrixXd  &mag_reference);

#endif
