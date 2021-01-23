/*
 *	Author: Mark MacGillivray (Based on Anna's simulator)
 *	Project: Dalhousie CubeSat
 *	SubSystem: ADCS
 *	Date: 1-21-2021
 *
 *	Description: Contains the function "pd" which is passed an angular velocity vector relative to ECI expressed in BF "w_EKF",
 *	a quaternion vector representing rotation from ECI to BF "q_EKF", a quaternion vector expressed in NP and returns a 3x1 voltage
 *	vector "Vc" relative to BF.
 */

#include "PDcontroller.h"
#include <math.h>

Eigen::Matrix<double, 4, 1> quatMultiply(const Eigen::Matrix<double, 4, 1>& q0, const Eigen::Matrix<double, 4, 1>& q1);
Eigen::Matrix<double, 3, 1> quatRotate(const Eigen::Matrix<double, 4, 1>& q, const Eigen::Matrix<double, 3, 1>& r);
Eigen::Matrix<double, 4, 1> quatConjugate(const Eigen::Matrix<double, 4, 1>& q);

Eigen::Matrix<double, 3, 1> pd(const Eigen::Matrix<double, 3, 1>& w_EKF, const Eigen::Matrix<double, 4, 1>& q_EKF, const Eigen::Matrix<double, 4, 1>& q_NPI) {

	//Find quaternion inverse.
	Eigen::Matrix<double, 4, 1> q_EKF_inv = quatConjugate(q_EKF); 

	//Orbital rate in NP.
	Eigen::Matrix<double, 3, 1> w_ECINP; w_ECINP << 0,
													2 * M_PI / T,
													0;

	//Find angular rate error, "w_hat".
	Eigen::Matrix<double, 3, 1> w_ECINP_rotated = quatRotate(q_EKF_inv, w_ECINP);
	Eigen::Matrix<double, 3, 1> w_hat = w_EKF - quatRotate(q_EKF, w_ECINP_rotated);
	
	//Find vector portion of quaternion error, "q_hat_vec".
	Eigen::Matrix<double, 4, 1> q_hat = quatMultiply(q_EKF_inv, q_NPI);
	Eigen::Matrix<double, 3, 1> q_hat_vec; q_hat_vec << q_hat(1, 0), q_hat(2, 0), q_hat(3, 0);
	
	//Find and return voltage vector expressed in BF.
	Eigen::Matrix<double, 3, 1> Vc = -Kp * q_hat_vec - Kd * w_hat;
	return Vc;
}

Eigen::Matrix<double, 4, 1> quatMultiply(const Eigen::Matrix<double, 4, 1>& q0, const Eigen::Matrix<double, 4, 1>& q1) {
	/****************************************
	Multiplies two quaternions together and returns their product using this algorithm:
	https://www.mathworks.com/help/aerotbx/ug/quatmultiply.html#mw_beba0414-8a76-47e8-9a3b-696cfdd1610f
	
	input: q0, q1
	output: quatProduct
	****************************************/
	double w0 = q0(0, 0);// <- must be the scalar value of quaternion 0.
	double x0 = q0(1, 0);
	double y0 = q0(2, 0);
	double z0 = q0(3, 0);

	double w1 = q1(0, 0);// <- must be the scalar value of quaternion 1.
	double x1 = q1(1, 0);
	double y1 = q1(2, 0);
	double z1 = q1(3, 0);

	Eigen::Matrix<double, 4, 1> quatProduct; quatProduct << -x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,
															x1* w0 + y1 * z0 - z1 * y0 + w1 * x0,
														   -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
															x1* y0 - y1 * x0 + z1 * w0 + w1 * z0;

	return quatProduct;
}

Eigen::Matrix<double, 3, 1> quatRotate(const Eigen::Matrix<double, 4, 1>& q, const Eigen::Matrix<double, 3, 1>& r) {
	/****************************************
	Takes quaternion "q" and 3x1 matrix "r" and returns "r" rotated by the quaternion using this algorithm:
	https://www.mathworks.com/help/aerotbx/ug/quatrotate.html#mw_2ebe648e-7e8e-4ce6-9bd5-a693da88b99b
	
	input: q, r
	output: rotatedInput
	****************************************/
		double q0 = q(0);
		double q1 = q(1);
		double q2 = q(2);
		double q3 = q(3);
		Eigen::Matrix<double, 3, 3> rotationMatrix; rotationMatrix << (1 - 2 * std::pow(q2, 2)) - 2 * std::pow(q3, 2), 2 * (q1*q2 + q0 * q3), 2 * (q1*q3 - q0 * q2),
																	   2 * (q1*q2 - q0 * q3), (1 - 2 * std::pow(q1, 2) - 2 * std::pow(q3, 2)), 2 * (q2*q3 + q0 * q1),
																	   2 * (q1*q3 + q0 * q2), 2 * (q2*q3 - q0 * q1), (1 - 2 * std::pow(q1, 2) - 2 * std::pow(q2, 2));
		
		Eigen::Matrix<double, 3, 1> rotatedInput = rotationMatrix * r;
		
		return rotatedInput;
	}

Eigen::Matrix<double, 4, 1> quatConjugate(const Eigen::Matrix<double, 4, 1>& q) {
	/****************************************
	Takes quaternion "q" and returns its conjugate by negating its vector portion.

	input: q
	output: q_conj
	****************************************/
	Eigen::Matrix<double, 4, 1> q_conj;
	q_conj(0, 0) =	q(0, 0);
	q_conj(1, 0) = -q(1, 0);
	q_conj(2, 0) = -q(2, 0);
	q_conj(3, 0) = -q(3, 0);
	
	return q_conj;
}