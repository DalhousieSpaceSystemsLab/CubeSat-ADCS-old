/*
 *	Author: Mark MacGillivray (Based on Dr. Bauer's algorithm)
 *	Project: Dalhousie CubeSat
 *	SubSystem: ADCS
 *  Date: 2020-02-13
 *
 *	Description: Contains the function "sun_vector_estimate" which takes the
 *  integer of sun sensors above threshold number "r", sun sensor intensity
 *  eigen matrix "y", eigen matrix of sun sensors above threshold number "sens",
 *  eigen matrix of sun sensor normals "H", and returns sun vector estimate as
 *	an eigen matrix from a body-fixed frame "s_hat_BF".
 */

#include "Sun_Vector_Estimate.h"

double norm1(Eigen::MatrixXd A) {
	/****************************************
	Takes eigen matrix "A" and returns its 1-norm.
	
	input: A
	output: norm1
	****************************************/

	//Sums the absolute values of each column.
	std::vector<double>sumOfCol(A.cols());
	for (int j = 0; j < A.cols(); j++) {
		for (int i = 0; i < A.rows(); i++) {
			sumOfCol[j] += abs(A(i, j));
		}
	}

	//Finds the largest sum out of all of the columns.
	double currentTopNum = 0;
	for (unsigned int j = 0; j < sumOfCol.size(); j++) {
		if (sumOfCol[j] > currentTopNum) {
			currentTopNum = sumOfCol[j];
		}
	}

	//Sets norm1 as the largest sum.
	double norm1 = currentTopNum;
	return norm1;
}

double rcond(Eigen::MatrixXd A) {
	/****************************************
	Acts like Matlab "rcond" function. Takes a square eigen matrix "A" and
	returns its reciprocal condition.
	
	input: A
	output: rcond
	****************************************/

	double rcond = 1 / (norm1(A)*norm1(A.inverse()));
	return rcond;
}

Eigen::MatrixXd sun_vector_estimate(int r, Eigen::MatrixXd y, Eigen::MatrixXd sens, Eigen::MatrixXd H) {
	/****************************************
	Takes the integer of sun sensors above threshold number "r", sun sensor intensity
	eigen matrix "y", eigen matrix of sun sensors above threshold number "sens",
	eigen matrix of sun sensor normals "H", and returns sun vector estimate as
 	an eigen matrix from a body-fixed frame "s_hat_BF".
	
	input: r, y, sens, H
	output: s_hat_BF
	****************************************/

	Eigen::MatrixXd s_hat_BF(3, 1);

	//If there are less than 3 sensors that can see the Sun, the estimate cannot be determined.
	if (r < 3) {
		s_hat_BF << 0,
					0,
					0;
	}
	else {

		//Remaps sun sensor intensity vector "y" to keep only values above intensity threshold.
		Eigen::MatrixXd tempY(sens.rows(), 1);
		for (int i = 0; i < sens.rows(); i++) {
			tempY(i, 0) = y(sens(i, 0) - 1, 0);
		}
		y = tempY;

		/*
		Remaps sun sensor normals vector "H" to keep only normals with sun sensor values
		above intensity threshold.
		*/
		Eigen::MatrixXd tempH(sens.rows(), 3);
		for (int i = 0; i < sens.rows(); i++) {
			for (int j = 0; j < 3; j++) {
				tempH(i, j) = H(sens(i, 0) - 1, j);
			}
		}
		H = tempH;

		/*
		Checks if reciprical condition of "H"'s conjugate transpose * "H" is well conditioned.
		If it's poorly conditioned, the sun vector estimate cannot be determined.
		*/
		if (rcond(H.conjugate().transpose()*H) > 1E-6) {
			//Determines sun vector estimate. 
			s_hat_BF = (H.conjugate().transpose()*H)
			.inverse()*H.conjugate().transpose()*y;
		}
		else {
			s_hat_BF << 0,
						0,
						0;
		}
	}

	//Returns sun vector estimate from body-fixed frame.
	return s_hat_BF;
}