/*
 *	Author: Mark MacGillivray (Based on Anna's simulator)
 *	Project: Dalhousie CubeSat
 *	SubSystem: ADCS
 *	Date: 1-21-2021
 */

#include "Eigen/Dense"

#ifndef PDCONTROLLER_H_
#define PDCONTROLLER_H_

#define Kp 1e-9 //Proportional gain in Nm, from Anna's thesis
#define Kd 2e-5 //Differential gain in Nms/rad, from Anna's thesis

#define M_PI 3.14159265358979323846264338327950288 //pi
#define T 5565 //period of ISS (LORIS) orbit in seconds

extern Eigen::Matrix<double, 3, 1> pd(const Eigen::Matrix<double, 3, 1>& w_est,
									  const Eigen::Matrix<double, 4, 1>& q_est,
									  const Eigen::Matrix<double, 4, 1>& q_NPI);

#endif /*PDCONTROLLER_H_*/