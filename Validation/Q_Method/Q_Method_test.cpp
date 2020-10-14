/*
 *	Author: Mark MacGillivray
 *	Project: Dalhousie CubeSat
 *	SubSystem: ADCS
 *	Date: 2020-10-13
 *
 *  Description: Tests Q Method algorithm by reading .txt
 *	files of input matrices b1, b2, b3, r1, r2 and prints
 *	a series of estimated attitude quaternions "q_est"
 *	to a .txt file called "q_est_cpp.txt". Each column of
 *	the inputs and outputs represents one iteration in a test
 *	of a total of 11 sets of inputs and outputs.
 *
 *	To be used with "Q_Method.h" and "Q_Method.cpp".
 *
 */
#include "Q_Method.h"
#include <fstream>
#include <iostream>
#include <iomanip>

int main() {
	std::ifstream in_b1, in_b2, in_b3, in_r1, in_r2;
	std::ofstream out_q_est;
	Eigen::MatrixXd b1(3, 11);
	Eigen::MatrixXd b2(3, 11);
	Eigen::MatrixXd b3(3, 11);
	Eigen::MatrixXd r1(3, 11);
	Eigen::MatrixXd r2(3, 11);
	Eigen::MatrixXd q_est(4, 11);
	
	// Read input values.
	in_b1.open("b1.txt");
	in_b2.open("b2.txt");
	in_b3.open("b3.txt");
	in_r1.open("r1.txt");
	in_r2.open("r2.txt");
	
	// Initialize input values.
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 11; j++) {
			 
			in_b1 >> b1(i, j);
			in_b2 >> b2(i, j);
			in_b3 >> b3(i, j);
			in_r1 >> r1(i, j);
			in_r2 >> r2(i, j);
		}
	}
  
	// Finds estimated attitude quaternion for each test
	// iteration and includes the result in the q_est matrix.
	for (int i = 0; i < 11; i++) {
		q_est.block<4, 1>(0, i) = q_method(b1.col(i), b2.col(i), b3.col(i), r1.col(i), r2.col(i));
	}

	// Prints q_est to a .txt file.
	out_q_est.open("q_est_cpp.txt");
	out_q_est << std::fixed << std::setprecision(15);
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 11; j++) {
			out_q_est << q_est(i, j);
		}
		out_q_est << "\n";
	}
	std::cout << "q_est results printed in q_est_cpp.txt\n";
}
