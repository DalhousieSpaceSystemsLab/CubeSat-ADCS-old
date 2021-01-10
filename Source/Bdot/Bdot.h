/*
 *	Author: Sean Smith (an implementation of Anna Wailands algorithm)
 *	Project: Dalhousie CubeSat
 *	SubSystem: ADCS
 *	Date: 2020-12-28
 *
 *  Description: Contains the function "Bdot" which takes (according to Anna):

	Eigen vector B = current noisy magentic field reading from IGRF -12 model in coordinates (X,Y,Z).
	Eigen vector B_1 = previous (1 second time step assumption) noisy magentic field reading from IGRF -12 model.
	Eigen vector fBdot_1 = previous filtered derivative of field (from feedback).
	Returns the required current magnetic moment as an Eigen vector.

 */

#ifndef Bdot_H_
#define Bdot_H_

#include <iostream>
#include <Eigen/Dense>
using namespace Eigen;

extern RowVectorXd Bdot(
	RowVectorXd B,
	RowVectorXd B_1,
	RowVectorXd fbdot_1
);

#endif /*Bdot_H_*/