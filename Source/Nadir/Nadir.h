/*
 *	Author: Mark MacGillivray (an implementation of Anna Wailand's algorithm)
 *	Project: Dalhousie CubeSat
 *	SubSystem: ADCS
 *	Date: 2020-05-21
 *
 *  Description: Contains the function "nadir" which takes node longitude
 *  (omega), right ascension of the ascending node (RAAN), inclination (i),
 *  true anomaly (tano) in degrees, and returns the nadir quaternion as
 *	a 1x4 matrix (q_NPI):
 */

#ifndef NADIR_H_
#define NADIR_H_

#include <Eigen/Dense>

extern Eigen::MatrixXd nadir(
	double omega,
	double RAAN,
	double i,
	double tano
);

#endif /*NADIR_H_*/
