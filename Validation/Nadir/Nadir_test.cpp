/*
 *	Author: Mark MacGillivray
 *	Project: Dalhousie CubeSat
 *	SubSystem: ADCS
 *	Date: 2020-10-13
 *
 *  Description: Tests nadir-finding algorithm by reading .txt
 *	files of inputs omega, RAAN, i, and tano in degrees and
 *	printing a 1x4 matrix of the attitude quaternion q_NPI
 *	to a .txt file called "q_NPI_cpp.txt".
 *
 *	To be used with "Nadir.h" and "Nadir.cpp".
 *
 */
#include "nadir.h"
#include <fstream>
#include <iostream>
#include <iomanip>

int main() {
	std::ifstream in_omega, in_RAAN, in_i, in_tano;
	std::ofstream out_q_NPI;

	double omega, RAAN, i, tano;
	Eigen::MatrixXd q_NPI(1, 4);
	
	// Read and initialize input values.
	in_omega.open("omega.txt");
	in_RAAN.open("RAAN.txt");
	in_i.open("i.txt");
	in_tano.open("tano.txt");

	in_omega >> omega;
	in_RAAN >> RAAN;
	in_i >> i;
	in_tano >> tano;

	// Find attitude quaternion q_NPI by using nadir().
	q_NPI = nadir(omega, RAAN, i, tano);
	
	// Print attitude quaternion q_NPI to .txt file.
	out_q_NPI.open("q_NPI_cpp.txt");
	out_q_NPI << std::fixed << std::setprecision(15);
	for (int i = 0; i < 4; i++) {
		out_q_NPI << q_NPI(0, i) << " ";
	}
	std::cout << "q_NPI results printed in q_NPI_cpp.txt\n";
}
