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
#include "../Errors.h"

ret_val SunEstimate(Eigen::MatrixXd intensity_matrix, Eigen::MatrixXd normals_matrix, Eigen::Vector3d  &sun_estimate);

#endif /*SUN_VECTOR_ESTIMATE_H_*/
