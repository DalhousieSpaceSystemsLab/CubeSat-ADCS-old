// Extended_Kalman_Filter.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <fstream>
#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/LU"	// needed to use .inverse function
#include "Eigen/SVD"
#include "Extended_Kalman_Filter.h"


using namespace std;

//#define EKF_structures_debug	// outputs constant initialization to screen
#define test					// used to test EFK model individually, generates text file x_hat_kk_cpp.txt to be used as input to MATLAB plotting inside testing folder

/* initialize constant matrices as 0's */
Eigen::Matrix3f J = Eigen::Matrix3f::Zero();	// define a 3x3 float matrix
Eigen::Matrix<float, 10, 10> Q = Eigen::Matrix<float, 10, 10>::Zero(); // assumed process noise covariance matrix Q
Eigen::Matrix<float, 7, 7> R = Eigen::Matrix<float, 7, 7>::Zero();	// assumed measurement noise covariance matrix R

float y[y_size_xaxis][input_size_yaxis] = { 0 };
float u[u_size_xaxis][input_size_yaxis] = { 0 };
float desired[desired_size_xaxis][input_size_yaxis] = { 0 };

void EKF_Driver();
void Initialize_EKF_structures(); 
void EKF_with_Gyro(const Eigen::Matrix<float, 10, 1>& x_hat_new, const Eigen::Matrix<float, 10, 10>& P_new,
	const Eigen::Matrix<float, 10, 1>& x_hat_prev, const Eigen::Matrix<float, 10, 10>& P_prev,
	const Eigen::Matrix<float, 3, 1>& u_new, const Eigen::Matrix<float, 7, 1>& y_new);
Eigen::Matrix3f skew(const Eigen::Matrix<float, 3, 1>& x);

#ifdef test
int main()
{
	EKF_Driver();
}
#endif

void Initialize_EKF_structures() {
	/************************************************************************
	Description: This function initializes matrices/constants and reads in 
	supporting text files used by Matlab version. 

	Author: Cathy
	input: none
	output:
	*************************************************************************/

	/* DECLARE VARIABLES */
	int i = 0, j = 0;	// counters
	ifstream input_y, input_u, input_desired;	// input text file streams

	Eigen::Matrix3f model_uncertainty = Eigen::Matrix3f::Zero();	//10 percent model uncertainty
	model_uncertainty.diagonal() << 1.1, 1.1, 0.9;

	/* INITIALIZATIONS OF CONSTANT GLOBAL MATRICCES */
	J.diagonal() << Ixx, Iyy, Izz;	// exact inertia matrix of rectangular prism
	J = J * model_uncertainty;

	/* assumed process noise covariance matrix Q which we give to Kalman Filter
	 since in practice we don't know this exactly, we can try to estimate
	 the values (here I multiply the actual values by 1.5)*/
	Q.diagonal() << phi_1_variance, phi_1_variance, phi_1_variance,
		phi_2_variance, phi_2_variance, phi_2_variance,
		phi_3_variance, phi_4_variance, phi_4_variance, phi_4_variance;
	Q = Q * 1.5;


	/* assumed measurement noise covariance matrix R which we give to Kalman Filter
	 since in practice we don't know this exactly, we can try to estimate
	 the values (here I multiply the actual values by 1.5)*/
	R.diagonal() << psi_1_variance, psi_1_variance, psi_1_variance,
		psi_2_variance, psi_2_variance, psi_2_variance,
		psi_3_variance;
	R = R * 1.5;

	input_y.open("y.txt");
	for (i = 0; i < y_size_xaxis; i++) {
		for (j = 0; j < input_size_yaxis; j++) {
			input_y >> y[i][j];
		}
	}
	/* READ IN TEST DATA THROUGH TXT FILES */
	input_u.open("u.txt");
	for (i = 0; i < u_size_xaxis; i++) {
		for (j = 0; j < input_size_yaxis; j++) {
			input_u >> u[i][j];
		}
	}

	input_desired.open("desired.txt");
	for (i = 0; i < desired_size_xaxis; i++) {
		for (j = 0; j < input_size_yaxis; j++) {
			input_desired >> desired[i][j];
		}
	}

	/* PRINT OUT INITIALIZED DATA VALUES */
#ifdef EKF_structures_debug
	cout << "J:\n" << J << '\n';
	cout << "Q:\n" << Q << '\n';
	cout << "R:\n" << R << '\n';
	cout << "\ny:\n";
	for (i = 0; i < y_size_xaxis; i++) {
		for (j = 0; j < input_size_yaxis; j++) {
			cout << y[i][j] << '\t';
		}
		cout << '\n';

	}
	cout << "\n\nu:\n";
	for (i = 0; i < u_size_xaxis; i++) {
		for (j = 0; j < input_size_yaxis; j++) {
			cout << u[i][j] << '\t';
		}
		cout << '\n';
	}

	cout << "\n\ndesired:\n";
	for (i = 0; i < desired_size_xaxis; i++) {
		for (j = 0; j < input_size_yaxis; j++) {
			cout << desired[i][j] << '\t';
		}
		cout << '\n';
	}
#endif
}

void EKF_Driver() {
	/************************************************************************
	Description: This function is modeled after the EKF_with_gyro_drv.m and
	calls both Initialize_EKF_structures() and EKF_with_gyro() to run a 
	simulation with equivalent results to the original Matlab version.

	Author: Cathy
	input: none
	output: none
	*************************************************************************/

	int k = 0, j = 0;


	ofstream x_hat_kk_results;

	Eigen::Matrix<float, 10, 1> x_hat_kk[100] = { Eigen::Matrix<float, 10, 1>::Zero() };
	Eigen::Matrix<float, 10, 10> P_kk[100] = { Eigen::Matrix<float, 10, 10>::Zero() };
	Eigen::Matrix<float, 3, 1> u_k_1 = Eigen::Matrix<float, 3, 1>::Zero();
	Eigen::Matrix<float, 7, 1> y_k_1 = Eigen::Matrix<float, 7, 1>::Zero();

	Initialize_EKF_structures();	// initalize and load in constant data structures for testing

	/* initial current state t = 0 */
	x_hat_kk[0] << 0.04, -0.02, -0.03, 0.6, 0.2, -0.1, 0.02, 0.05, -0.1, -0.15;
	
	for (k = 1; k < tmax / Ts; k++) {
		u_k_1 << u[0][k-1], u[1][k-1], u[2][k-1];
		y_k_1 << y[0][k], y[1][k], y[2][k], y[3][k], y[4][k], y[5][k], y[6][k];
		EKF_with_Gyro(x_hat_kk[k], P_kk[k], x_hat_kk[k-1], P_kk[k-1], u_k_1, y_k_1);
	}
#ifdef test
	for (k = 0; k < 100; k++) {
		cout << "\n\nx_hat_kk[" << k+1 << "]:\n" << x_hat_kk[k];
	}
	x_hat_kk_results.open("x_hat_kk_cpp.txt");
	for (j = 0; j < 10; j++) {
		for (k = 0; k < 100; k++) {
			x_hat_kk_results << x_hat_kk[k](j, 0) << "  ";
		}
		x_hat_kk_results << "\n";
	}
#endif
}

void EKF_with_Gyro(const Eigen::Matrix<float, 10, 1>& x_hat_new, const Eigen::Matrix<float, 10, 10>& P_new, 
	const Eigen::Matrix<float, 10, 1>& x_hat_prev, const Eigen::Matrix<float, 10, 10>& P_prev, 
	const Eigen::Matrix<float, 3, 1>& u_new, const Eigen::Matrix<float, 7, 1>& y_new) {
	/************************************************************************
	Description: This function is modeled after the EKF_with_gyro.m and
	executes the extended kalman filter calculations

	Author: Cathy
	input: x_hat_kk, P_kk, u_k_1, y_k_1

			NOTE: in order to pass matrices into functions they must be 
			constants. For the case of x_hat_kk and P_kk we can const_cast
			the const inputs in order to change the original variable.

	output: none
	*************************************************************************/

	Eigen::Matrix<float, 10, 1>& x_hat_new_mutable = const_cast<Eigen::Matrix<float, 10, 1>&> (x_hat_new);
	Eigen::Matrix<float, 10, 10>& P_new_mutable = const_cast<Eigen::Matrix<float, 10, 10>&> (P_new);

	/* use matrix.block properties to initialize w, q, and q0 components from x */
	Eigen::Matrix<float, 3, 1> w_k_1k_1 = x_hat_prev.block<3, 1>(0, 0);	// w_k_1k_1 = x_hat_prev(1:3)
	Eigen::Matrix<float, 3, 1> q_k_1k_1 = x_hat_prev.block<3, 1>(3, 0);	// q_k_1k_1 = x_hat_prev(4:6)
	Eigen::Matrix<float, 1, 1> q0_k_1k_1 = x_hat_prev.block<1, 1>(6, 0);	// q0_k_1k_1= x_hat_prev(7)
	
	/* state prediction x_hat_kk_1(10x1) */
	Eigen::Matrix<float, 10, 1> x_hat_kk_1 = Eigen::Matrix<float, 10, 1>::Zero();
	x_hat_kk_1.block<3, 1>(0, 0) = -J.inverse()*skew(w_k_1k_1)*(J*w_k_1k_1) + J.inverse()*u_new;
	x_hat_kk_1.block<3, 1>(3, 0) = -0.5*skew(w_k_1k_1)*q_k_1k_1 + (0.5*q0_k_1k_1*w_k_1k_1.transpose()).transpose();
	x_hat_kk_1.block<1, 1>(6, 0) = -0.5*w_k_1k_1.transpose()*q_k_1k_1;
	// x_hat_kk_1(8:10) = [0, 0, 0] (for which the values are already set)
	x_hat_kk_1 = Ts * x_hat_kk_1 + x_hat_prev;


	/* covariance prediction Pkk_1 from previous prediction P_k_1k_1 */
	// dF1_dw(3x3)
	Eigen::Matrix3f dF1_dw = Eigen::Matrix3f::Identity()-J.inverse()*(skew(w_k_1k_1)*J-skew(J*w_k_1k_1))*Ts;
	// dF2_dw(3x3)
	Eigen::Matrix3f dF2_dw = 0.5*(skew(q_k_1k_1) + q0_k_1k_1(0,0) * Eigen::Matrix3f::Identity())*Ts;						
	// dF2_dq(3x1)
	Eigen::Matrix3f dF2_dq = Eigen::Matrix3f::Identity() - 0.5*skew(w_k_1k_1)*Ts;
	// dF2_d0(3x1)
	Eigen::Matrix<float, 3, 1> dF2_d0 = 0.5*w_k_1k_1*Ts;		
	// dF3_dw(1x3)
	Eigen::Matrix<float, 1, 3> dF3_dw = -0.5*q_k_1k_1.transpose()*Ts;	
	// dF3_dq(1x3)
	Eigen::Matrix<float, 1, 3> dF3_dq = -0.5*w_k_1k_1.transpose()*Ts;		
	// dF3_dq0(1x1)
	Eigen::Matrix<float, 1, 1> dF3_dq0; dF3_dq0 << 1;

	Eigen::Matrix<float, 10, 10> F_k_1 = Eigen::Matrix<float, 10, 10>::Zero();
	/* initializing columns 0-2, rows 0-9 (index of 0) */
	F_k_1.block<3, 3>(0, 0) = dF1_dw;
	/* initializing columns 3-5, rows 0-9 (index of 0) */
	F_k_1.block<3, 3>(3, 0) = dF2_dw;
	F_k_1.block<3, 3>(3, 3) = dF2_dq;
	F_k_1.block<3, 1>(3, 6) = dF2_d0;
	/* initializing column 6, rows 0-9 (index of 0) */
	F_k_1.block<1, 3>(6, 0) = dF3_dw;
	F_k_1.block<1, 3>(6, 3) = dF3_dq;
	F_k_1.block<1, 1>(6, 6) = dF3_dq0;
	/* initializing columns 7-9, rows 0-9 (index of 0) */
	F_k_1.block<3, 3>(7, 7) = Eigen::Matrix3f::Identity();

	Eigen::Matrix<float, 10, 10> L_k_1 = Eigen::Matrix<float, 10, 10>::Identity()*Ts;

	// Covariance Prediction P_kk_1 10x10
	Eigen::Matrix<float, 10, 10> P_kk_1 = F_k_1 * P_prev*F_k_1.transpose() + L_k_1 * Q*L_k_1.transpose();

	/* Innovation (Measurement Residual) y_tilda_k(7x1) */
	Eigen::Matrix<float, 7, 10> H = Eigen::Matrix<float, 7, 10>::Zero();
	/* initializing columns 0-2, rows 0-9 (index of 0) */
	H.block<3, 3>(0, 0) = Eigen::Matrix3f::Identity();
	H.block<3, 3>(0, 7) = Eigen::Matrix3f::Identity();
	/* initializing columns 3-6, rows 0-9 (index of 0) */
	H.block<3, 3>(3, 3) = Eigen::Matrix3f::Identity();
	/* initializing column 7, rows 0-9 (index of 0) */
	H(6, 6) = 1.0;
	Eigen::Matrix<float, 7, 1> y_tilda_k = y_new - H * x_hat_kk_1;

	/* Innovation Covariance S_k 7x7 */
	Eigen::Matrix<float, 7, 7> S_k = H * P_kk_1*H.transpose() + R;

	/* Kalman Gain K_k 10x7 */
	Eigen::Matrix<float, 10, 7> K_k = Eigen::Matrix<float, 10, 7>::Zero();

	/* State Update x_hat_kk 10x1 */
	
	Eigen::JacobiSVD<Eigen::MatrixXf> svd(S_k);
	// calculate condition number (more deliable than determinant) to check invertibility of S_k
	float cond = svd.singularValues()(0) / svd.singularValues()(svd.singularValues().size() - 1);
	if (!isnan(cond)) {
		K_k = P_kk_1 * H.transpose()*S_k.inverse();
	}

	/* State Update x_hat_new 10x1 */
	x_hat_new_mutable = x_hat_kk_1 + K_k * y_tilda_k;

	Eigen::Matrix<float, 4, 1> q = x_hat_new_mutable.block<4, 1>(3, 0);	// isolate quaternion estimate
	q = q / q.norm();													// normalize quaterion estimate
	x_hat_new_mutable.block<4, 1>(3, 0) = q;							// update x_hat_kk with normalized quaternion

	/* Covariance Update P_kk 10x10 */
	P_new_mutable = (Eigen::Matrix<float, 10, 10>::Identity() - K_k*H)*P_kk_1;
}

Eigen::Matrix3f skew(const Eigen::Matrix<float, 3, 1>& x) {
	/************************************************************************
	Description: This function calculates skew-symmetric matrix of vector x

	Author: Cathy
	input: x (3x1 matrix desired to be skewed)
	output: result (3x3 matrix skew result of x)
	*************************************************************************/
	Eigen::Matrix3f result = Eigen::Matrix3f::Zero();
	result << 0, -x(2, 0), x(1, 0),
			x(2, 0), 0, -x(0, 0),
			-x(1, 0), x(0, 0), 0;
	return result;
}