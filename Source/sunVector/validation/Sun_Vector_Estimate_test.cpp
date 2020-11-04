/*
 *	Author: Mark MacGillivray
 *	Project: Dalhousie CubeSat
 *	SubSystem: ADCS
 *	Date: 2020-10-14
 *
 *  Description: Tests sun vector estimate algorithm by reading .txt
 *	files of input matrices y, H and prints a series of estimated attitude
 *	quaternions "s_hat_BF" to a .txt file called "s_hat_BF_cpp.txt". Each column of
 *	the inputs and outputs represents one iteration in a test of a total of
 *	11 sets of inputs and outputs.
 *
 *	To be used with "Sun_Vector_Estimate.h" and "Sun_Vector_Estimate.cpp".
 *
 */
#include "Sun_Vector_Estimate.h"
#include <fstream>
#include <iostream>
#include <iomanip>

int main() {
	std::ifstream in_y, in_H;
	std::ofstream out_s_hat_BF;
	
	Eigen::MatrixXd y(18, 11);
	Eigen::MatrixXd H(18, 3);

	// Read input values.
	in_y.open("y.txt");
	in_H.open("H.txt");

	// Initialize input values.
	for (int i = 0; i < 18; i++) {
		for (int j = 0; j < 11; j++) {
			in_y >> y(i, j);
		}
	}
	for (int i = 0; i < 18; i++) {
		for (int j = 0; j < 3; j++) {
			in_H >> H(i, j);
		}
	}

	// Finds sun vector estimate for each test iteration
	// and includes the result in the s_hat_BF matrix.
	out_s_hat_BF.open("s_hat_BF_cpp.txt");
	out_s_hat_BF << std::fixed << std::setprecision(15);
	for (int i = 0; i < 11; i++) {
		Eigen::MatrixXd yCol(18, 1);
		yCol = y.col(i);			
		for (int k = 0; k < 3; k++) {
			out_s_hat_BF << sun_vector_estimate(yCol, H)(k,0) << " ";
		}
		out_s_hat_BF << "\n";
	}
	std::cout << "s_hat_BF results printed in s_hat_BF_cpp.txt\n";
}
