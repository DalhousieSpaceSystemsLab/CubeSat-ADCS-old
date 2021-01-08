#pragma once
#include "Eigen/Dense"
#ifndef EXTENDED_KALMAN_FILTER_RW_H
#define EXTENDED_KALMAN_FILTER_RW_H

/*
 	Author: Mark MacGillivray (an implementation of Dr. Bauer's algorithm)
 	Project: Dalhousie CubeSat
	SubSystem: ADCS
 	Date: 2020-11-22
 
	Description: Contains the EKF class which can update x_hat_kk and P_kk vectors of the Extended Kalman Filter with asymmetrically positioned reaction wheels using the "update" member function, which is passed the u and y matricies.
  
	Each row of y corresponds to :
	1. ws1 scalar angular speed of RW#1 relative to BF
	2. ws2 scalar angular speed of RW#2 relative to BF
	3. ws3 scalar angular speed of RW#3 relative to BF
	4. wx CubeSat angular velocity relative to ECI expressed in BF % (x component) rad / s
	5. wy CubeSat angular velocity relative to ECI expressed in BF
	(y component) rad / s
	6. wz CubeSat angular velocity relative to ECI expressed in BF
	(z component) rad / s
	7. q1 element 1 of vector portion of quaternion representing rotation from ECI to BF
	8. q2 element 2 of vector portion of quaternion representing rotation from ECI to BF
	9. q3 element 3 of vector portion of quaternion representing rotation from ECI to BF
	10. q0 scalar element quaternion representing rotation from ECI to BF

	Each row of u corresponds to :
	1. gx applied external torque x(in BF) Nm
	2. gy applied external torque y(in BF) Nm
	3. gz applied external torque z(in BF) Nm
	4. ga1 applied scalar torque from motor on RW#1 Nm
	5. ga2 applied scalar torque from motor on RW#2 Nm
	6. ga3 applied scalar torque from motor on RW#3 Nm

	x_hat_kk and P_kk can be accessed by the member functions "get_x_hat_kk" and "get_P_kk".
 */

class EKF {

public:
	EKF(Eigen::Matrix<double, 13, 1> &x_hat_kk_initial);
	void update(const Eigen::Matrix<double, 6, 1> &u, const Eigen::Matrix<double, 10, 1> &y);
	void set_x_hat_kk(const Eigen::Matrix<double, 13, 1> &x_hat_kk);
	const Eigen::Matrix<double, 13, 1> get_x_hat_kk();
	void set_P_kk(const Eigen::Matrix<double, 13, 13>& P_kk);
	const Eigen::Matrix<double, 13, 13> get_P_kk();
	
private:
		Eigen::Matrix<double, 13, 1> x_hat_kk;
		Eigen::Matrix<double, 13, 13> P_kk = Eigen::Matrix<double, 13, 13>::Zero();
		void initialize_EKF_structures();
		void EKF_with_RW(const Eigen::Matrix3d& J, const Eigen::Matrix3d& Jnew_inv, double Is, const Eigen::Matrix<double, 3, 1>& a1, const Eigen::Matrix<double, 3, 1>& a2, const Eigen::Matrix<double, 3, 1>& a3,
			const Eigen::Matrix<double, 10, 10>& R, const Eigen::Matrix<double, 13, 13>& Q, const double Ts, const Eigen::Matrix<double, 3, 1>& g_k_1,
			const Eigen::Matrix<double, 3, 1>& gai_k_1, const Eigen::Matrix<double, 10, 1>& y_k, const Eigen::Matrix<double, 13, 1>& x_hat_k_1, const Eigen::Matrix<double, 13, 13>& P_k_1);
		Eigen::Matrix3d skew(const Eigen::Matrix<double, 3, 1>& x);
};
#endif //EXTENDED_KALMAN_FILTER_RW_H