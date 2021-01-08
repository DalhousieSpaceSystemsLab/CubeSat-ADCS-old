#include "Extended_Kalman_Filter_RW.h"
#include <fstream>
#include <iostream>
#include <iomanip>
int main() {
	
	/*	This is a file used to test the useage of the EKF. It takes the 500 sets of u and y inputs provided 
		by Dr. Bauer and returns x_hat_kk_cpp.txt and P_kk_cpp.txt to be validated with the corresponding Matlab code.
	*/
	const int tests = 500;

	Eigen::Matrix<double, 6, 1> u[tests] = {Eigen::Matrix<double, 6, 1>::Zero()};
	Eigen::Matrix<double, 10, 1> y[tests] = {Eigen::Matrix<double, 10, 1>::Zero()};
	
	Eigen::Matrix<double, 13, 1> x_hat_kk_initial = Eigen::Matrix<double, 13, 1>::Zero();
	Eigen::Matrix<double, 13, 1> x_hat_kk_sets[tests] = { Eigen::Matrix<double, 13, 1>::Zero() };
	Eigen::Matrix<double, 13, 13> P_kk_sets[tests] = { Eigen::Matrix<double, 13, 13>::Zero() };

	//Initial x_hat_kk value.
	x_hat_kk_initial << -.02, .04, .01, -.01, .01, .002, .5, -.25, .3, .8, .1, -.1, .15;

	std::ifstream in_u, in_y;
	in_u.open("u.txt");
	in_y.open("y.txt");

	std::ofstream x_hat_kk_cpp, P_kk_cpp;
	x_hat_kk_cpp.open("x_hat_kk_cpp.txt");
	P_kk_cpp.open("P_kk_cpp.txt");

	//constructing the ekf.
	EKF ekf(x_hat_kk_initial);
	
	//Populating u and y matrices.
	for (int i = 0; i < tests; i++) {
	for (int j = 0; j < 6; j++) {
			in_u >> u[i](j, 0);
		}
	}
	for (int i = 0; i < tests; i++) {
		for (int j = 0; j < 10; j++) {
			in_y >> y[i](j, 0);
		}
	}

	//Populating x_hat_kk sets with values given by updating the ekf with sets of u and y.
	x_hat_kk_sets[0] = ekf.get_x_hat_kk();
	for (int i = 1; i < tests; i++) {
		ekf.update(u[i-1], y[i]);
		x_hat_kk_sets[i] = ekf.get_x_hat_kk();
		P_kk_sets[i] = ekf.get_P_kk();
	}

	//Writing all x_hat_kk sets to the x_hat_kk_cpp.txt file for validating with Matlab code.
	x_hat_kk_cpp << std::fixed << std::setprecision(15);
	for (int i = 0; i < 13; i++) {
		for (int j = 0; j < tests; j++) {
			x_hat_kk_cpp << x_hat_kk_sets[j](i, 0) << " ";
		}
		x_hat_kk_cpp << "\n";
	}

	//Writing all P_kk sets to the P_kk_cpp.txt file for validating with Matlab code.
	P_kk_cpp << std::fixed << std::setprecision(15);
	for (int i = 0; i < 13; i++) {
		for (int j = 0; j < tests; j++) {
			for (int k = 0; k < 13; k++) {
			P_kk_cpp << P_kk_sets[j](i, k) << " ";
			}
		}
		P_kk_cpp << "\n";
	}
}