/*
 *	Author: Mark MacGillivray
 *	Project: Dalhousie CubeSat
 *	SubSystem: ADCS
 *	Date: 2020-02-13
 *
 *
 *
 */

#ifndef SUN_VECTOR_ESTIMATE_H_
#define SUN_VECTOR_ESTIMATE_H_

#include <Eigen/Dense>
#include <vector>

extern Eigen::MatrixXd sun_vector_estimate(
	Eigen::MatrixXd y,
	Eigen::MatrixXd H
);

#endif /*SUN_VECTOR_ESTIMATE_H_*/
