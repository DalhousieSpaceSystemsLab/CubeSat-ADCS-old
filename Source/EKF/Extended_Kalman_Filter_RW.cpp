// Extended_Kalman_Filter_RW.cpp : This file contains the 'main' function. Program execution begins and ends there.

#include <iostream>
#include <fstream>
#include <iomanip>
#include <math.h>
#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/LU"	// needed to use .inverse function
#include "Eigen/SVD"
#include "Extended_Kalman_Filter_RW.h"

/* INITIALIZATIONS OF CONSTANTS */
/* sizes of input arrays */
//#define M_PI 3.14159265358979323846  /* pi */

//dimensions of 2U rectangular prism R[m](a = length, b = height, c = depth)
const double a = .1;	// m
const double b = .1;	// m
const double c = .2;	// m

/* dimension of cylindrical - shaped RW rotor Wi
 THESE DIMENSIONS WILL NOT BE NEEDED IN THE FINAL DESIGN SINCE THE ROTOR
 WILL LIKELY NOT BE A SIMPLE CYLINDER
 */

#define r_rotor .02	// m, radius
#define l_rotor .01	// m, width


Eigen::Matrix<double, 3, 1> a1_b = (Eigen::Matrix<double, 3, 1>() << 1, 0, 0).finished();
Eigen::Matrix<double, 3, 1> a2_b = (Eigen::Matrix<double, 3, 1>() << 0, 1, 0).finished();
Eigen::Matrix<double, 3, 1> a3_b = (Eigen::Matrix<double, 3, 1>() << 0, 0, 1).finished();

//mass of CubeSat rigid body R(without RW rotors)
//VALUE FOR m_R WILL NEED TO BE ADJUSTED TO MATCH FINAL DESIGN
#define m_R 3.7999999999999976 // kg, value from SolidWorks

//mass of cylindrical - shaped RW rotor Wi
//VALUE FOR m_W WILL NEED TO BE ADJUSTED TO MATCH FINAL DESIGN
#define rho_W 1000 // kg / m ^ 3, density of rotor Wi
const double m_W = atan(1) * 4.0 * pow(r_rotor, 2) * l_rotor * rho_W; // kg, mass of rotor Wi

// VALUE FOR J_R WILL NEED TO BE ADJUSTED TO MATCH FINAL DESIGN

Eigen::Matrix3d J_R = (Eigen::Matrix3d() << 0.035945701754385918, 0.0014210526315789342, 0.0031263157894736802,
	0.0014210526315789342, 0.039761491228070128, 0.0023842105263157877,
	0.0031263157894736802, 0.0023842105263157877, 0.04825438596491223
	).finished();

// location of COM of CubeSat R(without rotors) from point A expressed in Fb
// VALUES FOR rAR_b, rAW1_b, rAW2_band rAW3_b WILL NEED TO BE ADJUSTED TO
// MATCH FINAL DESIGN

Eigen::Matrix<double, 3, 1> rAR_b = (Eigen::Matrix<double, 3, 1>() << -0.023684210526315773,
	-0.015789473684210509,
	0.14026315789473687
	).finished(); // m
// location of COM of W1 from point A expressed in Fb

Eigen::Matrix<double, 3, 1> rAW1_b = (Eigen::Matrix<double, 3, 1>() << 3.0 / 4.0 * a / 2.0,
	b / 4.0,
	3.0 / 4.0 * c
	).finished(); // m
// location of COM of W2 from point A expressed in Fb
Eigen::Matrix<double, 3, 1> rAW2_b = (Eigen::Matrix<double, 3, 1>() << 0,
	3.0 / 4.0 * b / 2,
	3.0 / 4.0 * c
	).finished(); // m

// location of COM of W3 from point A expressed in Fb
Eigen::Matrix<double, 3, 1> rAW3_b = (Eigen::Matrix<double, 3, 1>() << 0,
	0,
	7.0 / 8.0 * c
	).finished(); // m

// location of O(COM of combined R1& R2& W1& W2& W3 from point A) expressed in Fb
Eigen::Matrix<double, 3, 1> rAO_b = (1 / (m_R + 3 * m_W)) * (m_R * rAR_b + m_W * rAW1_b + m_W * rAW2_b + m_W * rAW3_b);

Eigen::Matrix<double, 3, 1> r_b = rAR_b - rAO_b;// location of COM of R(without rotors) from O
Eigen::Matrix<double, 3, 1> b1_b = rAW1_b - rAO_b; // location of COM of W1 from O
Eigen::Matrix<double, 3, 1> b2_b = rAW2_b - rAO_b; // location of COM of W2 from O
Eigen::Matrix<double, 3, 1> b3_b = rAW3_b - rAO_b; // location of COM of W3 from O

// use Parallel Axis Theorem to calculate moment of inertia of R wrt O
// wrt Fb axes, kgm ^ 2

Eigen::Matrix3d J_b = J_R + m_R * (pow(r_b.norm(), 2) * Eigen::Matrix3d::Identity() - r_b * r_b.transpose());

// moment of inertia of W wrt W's COM about its axis of symmetry, kgm^2
// VALUE for Is WILL NEED TO BE ADJUSTED TO MATCH FINAL DESIGN
const double Is = 1.0 / 2.0 * m_W * pow(r_rotor, 2);

// moment of inertia of W wrt W's COM about any transverse axis, kgm^2
// VALUE for It WILL NEED TO BE ADJUSTED TO MATCH FINAL DESIGN
const double It = 1.0 / 12.0 * m_W * (3.0 * pow(r_rotor, 2) + pow(l_rotor, 2));

// moment of inertia of W1 wrt W's COM wrt Fb axes
Eigen::Matrix3d IW1_b = It * Eigen::Matrix3d::Identity() + (Is - It) * (a1_b * a1_b.transpose());
// moment of inertia of W1 wrt W's COM wrt Fb axes
Eigen::Matrix3d IW2_b = It * Eigen::Matrix3d::Identity() + (Is - It) * (a2_b * a2_b.transpose());
// moment of inertia of W1 wrt W's COM wrt Fb axes
Eigen::Matrix3d IW3_b = It * Eigen::Matrix3d::Identity() + (Is - It) * (a3_b * a3_b.transpose());

Eigen::Matrix3d J = J_b + IW1_b + IW2_b + IW3_b + m_W * (pow(b1_b.norm(), 2) * Eigen::Matrix3d::Identity() - b1_b * b1_b.transpose()) + m_W * (pow(b2_b.norm(), 2) * Eigen::Matrix3d::Identity() - b2_b * b2_b.transpose()) + m_W * (pow(b3_b.norm(), 2) * Eigen::Matrix3d::Identity() - b3_b * b3_b.transpose());

// can add model uncertainty for KF
#define model_uncertainty 1.05 // 5% uncertainty
Eigen::Matrix3d J_KF = J * model_uncertainty;

Eigen::Matrix3d Jnew_KF = J_KF - Is * (a1_b * a1_b.transpose()) - Is * (a2_b * a2_b.transpose()) - Is * (a3_b * a3_b.transpose());
Eigen::Matrix3d Jnew_inv_KF = Jnew_KF.inverse();

const double Ts = 1;

/* Band-Limited White Gaussian Noise
	variance of noise signal
	standard deviation is sqrt(variance of noise signal)
	68 percent of the generated noise is within plus/minus one standard deviation
	process noise added to equations of motion written in state space
	x_dot = .... + process noise */

	// variance of process noise on OMEGA_dot used to generate test data
#define phi_0_variance  1.0e-5
// variance of process noise on w_dot used to generate test data
#define phi_1_variance	1.0e-8
// variance of process noise on q_dot used to generate test data
#define phi_2_variance	1.0e-5
// variance of process noise on q0_dot used to generate test data
#define phi_3_variance	1.0e-5
// variance of process noise on beta_dot used to generate test data
#define phi_4_variance	1.0e-4

// variance of measurement noise on OMEGA used to generate test data
#define psi_0_variance  1.0e-2
// variance of measurement noise on w used to generate test data
#define psi_1_variance	1.0e-5
// variance of measurement noise on q used to generate test data
#define psi_2_variance	1.0e-2
// variance of measurement noise on q0 used to generate test data
#define psi_3_variance	1.0e-2

//#define EKF_structures_debug	// outputs constant initialization to screen
#define test					// used to test EFK model individually, generates text file x_hat_kk_cpp.txt and
								// P_kk_cpp.txt to be used as input to MATLAB plotting inside testing folder

/* initialize constant matrices as 0's */
//Eigen::Matrix3d J = Eigen::Matrix3d::Zero();	// Look at this closer, it's in header file.
Eigen::Matrix3d Jw = Eigen::Matrix3d::Zero();	// define a 3x3 double matrix
Eigen::Matrix<double, 13, 13> Q = Eigen::Matrix<double, 13, 13>::Zero(); // assumed process noise covariance matrix Q
Eigen::Matrix<double, 10, 10> R = Eigen::Matrix<double, 10, 10>::Zero();	// assumed measurement noise covariance matrix R

EKF::EKF(Eigen::Matrix<double, 13, 1> &x_hat_kk_initial) {
	//P_kk_old = Eigen::Matrix<double, 13, 13>::Zero();
	set_x_hat_kk(x_hat_kk_initial);
}

void EKF::initialize_EKF_structures() {
	/************************************************************************
	Description: This function initializes matrices/constants.
	Author: Mark, Cathy
	input: none
	output: none
	*************************************************************************/

	/* DECLARE VARIABLES */
	int i = 0, j = 0;	// counters
	
	/*	assume Reaction Wheels are cylinders concentrated at the COM of the
		satellite
	*/
	double r = 0.02; //m
	double h = 0.01; //m
	double rho = 8730.0; //kg/m^3
	double mw = atan(1) * 4 * pow(r, 2) * h * rho; //kg
	
	//Add RW moments of inertia about non - spin axes to satellite inertia:
	double Iwt = mw * (3.0f * pow(2 * r, 2) + 4.0 * pow(h, 2)) / 48.0; //transverse moment of inertia of RWs
	
	J.diagonal() << J(0,0) + 2.0*Iwt, J(1,1) + 2.0*Iwt, J(2,2) + 2.0*Iwt; //add effect of Iwt for y and z, x and z, x and y RWs

	//Eigen::Matrix3d model_uncertainty = Eigen::Matrix3d::Zero();	//10 percent model uncertainty
	//model_uncertainty.diagonal() << 1.1, 1.1, 0.9;

	//J = J * model_uncertainty; since in practice we do not have an exact model
	//						     of the actual satellite we will assume some uncertainty

	/* assumed process noise covariance matrix Q which we give to Kalman Filter
	 since in practice we don't know this exactly, we can try to estimate
	 the values */
	Q.diagonal() << phi_0_variance, phi_0_variance, phi_0_variance,
					phi_1_variance, phi_1_variance, phi_1_variance,
					phi_2_variance, phi_2_variance, phi_2_variance,
					phi_3_variance, phi_4_variance, phi_4_variance, phi_4_variance;
	//Q = Q * 1.5;

	/* assumed measurement noise covariance matrix R which we give to Kalman Filter
	 since in practice we don't know this exactly, we can try to estimate
	 the values */
	R.diagonal() << psi_0_variance, psi_0_variance, psi_0_variance,
		psi_1_variance, psi_1_variance, psi_1_variance,
		psi_2_variance, psi_2_variance, psi_2_variance,
		psi_3_variance;
}
void EKF::update(const Eigen::Matrix<double, 6, 1>& u, const Eigen::Matrix<double, 10, 1>& y) {
	/************************************************************************
	Description: This function updates the x_hat_kk and P_kk values using column vectors u and y.
	/*
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

	Author: Mark
	input: u and y vectors
	output: none
	*************************************************************************/
	
	int k = 0, j = 0;

	Eigen::Matrix<double, 3, 1> g_k_1 = Eigen::Matrix<double, 3, 1>::Zero();
	Eigen::Matrix<double, 3, 1> gai_k_1 = Eigen::Matrix<double, 3, 1>::Zero();
	Eigen::Matrix<double, 10, 1> y_k = Eigen::Matrix<double, 10, 1>::Zero();
	Eigen::Matrix<double, 13, 1> x_hat_k_1 = Eigen::Matrix<double, 13, 1>::Zero();

	initialize_EKF_structures();	// initalize and load in constant data structures for testing

		g_k_1 << u[0], u[1], u[2];		
		gai_k_1 << u[3], u[4], u[5];
		y_k << y[0], y[1], y[2], y[3], y[4], y[5], y[6], y[7], y[8], y[9];
		EKF_with_RW(J_KF, Jnew_inv_KF, Is, a1_b, a2_b, a3_b, R, Q, Ts, g_k_1, gai_k_1, y_k, x_hat_kk, P_kk);
}

void EKF::EKF_with_RW(const Eigen::Matrix3d& J, const Eigen::Matrix3d& Jnew_inv, double Is, const Eigen::Matrix<double, 3, 1>& a1, const Eigen::Matrix<double, 3, 1>& a2, const Eigen::Matrix<double, 3, 1>& a3,
	const Eigen::Matrix<double, 10, 10>& R, const Eigen::Matrix<double, 13, 13>& Q, const double Ts, const Eigen::Matrix<double, 3, 1>& g_k_1,
	const Eigen::Matrix<double, 3, 1>& gai_k_1, const Eigen::Matrix<double, 10, 1>& y_k, const Eigen::Matrix<double, 13, 1>& x_hat_k_1, const Eigen::Matrix<double, 13, 13>& P_k_1) {
	/*************************************************************************
Description: This function is used to find values for x_hat_kk and P_kk using the following inputs. 
Author : Mark
input:
% J         = inertia matrix of entire system about O (at COM) taken wrt BF axes, kgm^2
% Jnew_inv  = inverse of portion of J used in angular acceleration equation,1/kgm^2
% Is        = scalar inertia value of RW rotor about the COM of the rotor
%             taken wrt to its spin axis, kgm^2
% ai        = spin axis direction (unit vector) of RWi expressed in BF, i=1,2,3
% R         = measurement noise covariance matrix
% Q         = process noise covariance matrix
% Ts        = sample time (s)
% g_k_1     = previous (k-1) external input torques applied to satellite expressed in BF, Nm
% gai_k_1   = scalar previous (k-1) input axial torque on RWi from motor i, Nm
% y_k       = current (k) measurement vector
% x_hat_k_1 = previous (k-1) state estimate
% P_k_1     = previous (k-1) covariance estimate

NOTE : in order to pass matrices into functions they must be
constants.For the case of x_hat_kkand P_kk we can const_cast
the const inputs in order to change the original variable.
output: none
	*************************************************************************/
	double ha1_k_1 = x_hat_k_1[0];
	double ha2_k_1 = x_hat_k_1[1];
	double ha3_k_1 = x_hat_k_1[2];
	Eigen::Matrix<double, 3, 1> w_k_1;
	w_k_1[0] = x_hat_k_1[3];
	w_k_1[1] = x_hat_k_1[4];
	w_k_1[2] = x_hat_k_1[5];
	Eigen::Matrix<double, 3, 1> q_k_1;
	q_k_1[0] = x_hat_k_1[6];
	q_k_1[1] = x_hat_k_1[7];
	q_k_1[2] = x_hat_k_1[8];
	double q0_k_1 = x_hat_k_1[9];

	// Convert RWi angular momentum to RWi angular velocity
	double ws1_k_1 = ha1_k_1 / Is - a1.transpose() * w_k_1;
	double ws2_k_1 = ha2_k_1 / Is - a2.transpose() * w_k_1;
	double ws3_k_1 = ha3_k_1 / Is - a3.transpose() * w_k_1;

	// separate RWi torques
	Eigen::Matrix<double, 1, 1> ga1_k_1;
	Eigen::Matrix<double, 1, 1> ga2_k_1;
	Eigen::Matrix<double, 1, 1> ga3_k_1;
	ga1_k_1 << gai_k_1(0);
	ga2_k_1 << gai_k_1(1);
	ga3_k_1 << gai_k_1(2);

	Eigen::Matrix<double, 13, 1> x_hat_k_k_1 = Eigen::Matrix<double, 13, 1>::Zero();
	Eigen::Matrix<double, 13, 1> A = Eigen::Matrix<double, 13, 1>::Zero();
	
	A.block<1, 1>(0, 0) = ga1_k_1;
	A.block<1, 1>(1, 0) = ga2_k_1;
	A.block<1, 1>(2, 0) = ga3_k_1;
	A.block<3, 1>(3, 0) = Jnew_inv * (-skew(w_k_1) * (J * w_k_1 + a1 * Is * ws1_k_1 + a2 * Is * ws2_k_1 + a3 * Is * ws3_k_1) - (a1 * ga1_k_1 + a2 * ga2_k_1 + a3 * ga3_k_1) + g_k_1);;
	A.block<3, 1>(6, 0) = -0.5 * skew(w_k_1) * q_k_1 + 0.5 * q0_k_1 * w_k_1;
	A.block<1, 1>(9, 0) = -0.5 * w_k_1.transpose()*q_k_1;
	A.block<3, 1>(10, 0) = Eigen::Matrix<double, 3, 1>::Zero();
	x_hat_k_k_1 = x_hat_k_1 + A * Ts;
	
	Eigen::Matrix3d dF1_dw = Eigen::Matrix3d::Identity() - Jnew_inv * (skew(w_k_1) * J - skew(J * w_k_1 + a1 * Is * ws1_k_1 + a2 * Is * ws2_k_1 + a3 * Is * ws3_k_1)) * Ts;
	Eigen::Matrix3d dF2_dw = 0.5 * (skew(q_k_1) + q0_k_1 * Eigen::Matrix3d::Identity()) * Ts;
	Eigen::Matrix3d dF2_dq = Eigen::Matrix3d::Identity() - 0.5 * skew(w_k_1) * Ts;
	Eigen::Matrix<double, 3, 1> dF2_dq0 = 0.5 * w_k_1 * Ts;
	
	Eigen::Matrix<double, 1, 3> dF3_dw = -0.5 * q_k_1.transpose()*Ts;
	Eigen::Matrix<double, 1, 3> dF3_dq = -0.5 * w_k_1.transpose()*Ts;
	Eigen::Matrix<double, 1, 1> dF3_dq0;
	dF3_dq0 << 1.0;
	Eigen::Matrix<double, 1, 1> one;
	one << 1.0;

	Eigen::Matrix<double, 13, 13> F_k_1 = Eigen::Matrix<double, 13, 13>::Zero();

	 F_k_1.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
	 F_k_1.block<1, 1>(9, 0) << 0;
	 F_k_1.block<3, 3>(3, 3) = dF1_dw;
	 F_k_1.block<3, 3>(6, 3) = dF2_dw;
	 F_k_1.block<1, 3>(9, 3) = dF3_dw;
	 F_k_1.block<3, 3>(6, 6) = dF2_dq;
	 F_k_1.block<1, 3>(9, 6) = dF3_dq;
	 F_k_1.block<3, 1>(6, 9) = dF2_dq0;
	 F_k_1.block<1, 1>(9, 9) = dF3_dq0;
	 F_k_1.block<3, 3>(10, 10) = Eigen::Matrix3d::Identity();
	
	Eigen::Matrix<double, 13, 13> L_k_1 = Eigen::Matrix<double, 13, 13>::Identity() * Ts; 
	
	Eigen::Matrix<double, 13, 13> P_k_k_1 = (F_k_1) * P_k_1 * (F_k_1).transpose() + L_k_1*Q*L_k_1.transpose();
		 
	Eigen::Matrix<double, 10, 13> H = Eigen::Matrix<double, 10, 13>::Zero();
	H.block<1, 1>(0, 0) << 1 / Is;
	H.block<1, 1>(1, 1) << 1 / Is;
	H.block<1, 1>(2, 2) << 1 / Is;
	H.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity();
	H.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity();
	H.block<1, 1>(9, 9) << 1.0;
	H.block<1, 1>(0, 3) << -1.0;
	H.block<1, 1>(1, 4) << -1.0;
	H.block<1, 1>(2, 5) << -1.0;
	H.block<3, 3>(3, 10) = Eigen::Matrix3d::Identity();
	
	Eigen::Matrix<double, 10, 1> y_tilda_k = y_k - H * x_hat_k_k_1;
	Eigen::Matrix<double, 10, 10> S_k = H * P_k_k_1 * H.transpose() + R;
	Eigen::JacobiSVD<Eigen::MatrixXd> svd(S_k);
	
	// calculate condition number (more delible than determinant) to check invertibility of S_k
	double cond = svd.singularValues()(0) / svd.singularValues()(svd.singularValues().size() - 1);
	
	Eigen::Matrix<double, 13, 10> K_k;
	if (isnan(cond)) { //Test this
		K_k = Eigen::Matrix<double, 13, 10>::Zero();
	}
	else {
		K_k = P_k_k_1 * H.transpose()*S_k.inverse();
	}
	Eigen::Matrix<double, 13, 1> x_hat_kk = x_hat_k_k_1 + K_k * y_tilda_k;
	
	Eigen::Matrix<double, 4, 1> q;
	q << x_hat_kk[6], x_hat_kk[7], x_hat_kk[8], x_hat_kk[9];
	q = q / q.norm();
	x_hat_kk.block<4, 1>(6, 0) = q;

	double ha1_kk = x_hat_kk[0];
	double ha2_kk = x_hat_kk[1];
	double ha3_kk = x_hat_kk[2];
	Eigen::Matrix<double, 3, 1> w_kk;
	w_kk << x_hat_kk[3], x_hat_kk[4], x_hat_kk[5];

	double ws1_kk = ha1_kk / Is - a1.transpose()*w_kk;
	double ws2_kk = ha2_kk / Is - a2.transpose()*w_kk;
	double ws3_kk = ha3_kk / Is - a3.transpose()*w_kk;

	Eigen::Matrix<double, 3, 1> wsi_kk;
	wsi_kk << ws1_kk, ws2_kk, ws3_kk;

	Eigen::Matrix<double, 13, 13> P_kk = (Eigen::Matrix<double, 13, 13>::Identity() - K_k * H) * P_k_k_1;

	set_x_hat_kk(x_hat_kk);
	set_P_kk(P_kk);
	}

Eigen::Matrix3d EKF::skew(const Eigen::Matrix<double, 3, 1>& x) {
	/************************************************************************
	Description: This function calculates skew-symmetric matrix of vector x
	Author: Cathy
	input: x (3x1 matrix desired to be skewed)
	output: result (3x3 matrix skew result of x)
	*************************************************************************/
	Eigen::Matrix3d result = Eigen::Matrix3d::Zero();
	result << 0, -x(2, 0), x(1, 0),
		x(2, 0), 0, -x(0, 0),
		-x(1, 0), x(0, 0), 0;
	return result;
}
void EKF::set_x_hat_kk(const Eigen::Matrix<double, 13, 1>& x_hat_kk) {
	this->x_hat_kk = x_hat_kk;
}

const Eigen::Matrix<double, 13, 1> EKF::get_x_hat_kk() {
	return x_hat_kk;
}

void EKF::set_P_kk(const Eigen::Matrix<double, 13, 13>& P_kk) {
	this->P_kk = P_kk;
}

const Eigen::Matrix<double, 13, 13> EKF::get_P_kk() {
	return P_kk;
}