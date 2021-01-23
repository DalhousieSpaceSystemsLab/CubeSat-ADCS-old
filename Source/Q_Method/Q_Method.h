/*
 *	Author: Mark MacGillivray (an implementation of Dr. Bauer's algorithm)
 *	Project: Dalhousie CubeSat
 *	SubSystem: ADCS
 *	Date: 2020-05-17
 *
 *  Description: Contains the function "q_method" which takes (according to Dr Bauer):
   
	Eigen matrix b1 = components of Sun vector expressed in Body-Fixed frame (from Diodes).
	Eigen matrix b2 = components of magnetic field vector A expressed in Body-Fixed frame.
	(from magnetometer A).
	Eigen matrix b3 = components of magnetic field vector B expressed in Body-Fixed frame
	(from magnetometer B).
	Eigen matrix r1 = components of Sun vector expressed in ECI frame
	(from theory).
	Eigen matrix r2 = components of magnetic field vector expressed in ECI frame (from theory).
	Returns the estimated attitude quaternion as an eigen matrix.
 */

#ifndef Q_METHOD_H_
#define Q_METHOD_H_

#include <Eigen/Dense>

extern Eigen::Vector4d q_method(
	Eigen::Vector3d b1,
	Eigen::Vector3d b2,
	Eigen::Vector3d b3,
	Eigen::Vector3d r1,
	Eigen::Vector3d r2
);

#endif /*Q_METHOD_H_*/