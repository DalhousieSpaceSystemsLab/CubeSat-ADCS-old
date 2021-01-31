/*
 *	Author: Mark MacGillivray (Based on Dr. Bauer's algorithm)
 *	Project: Dalhousie CubeSat
 *	SubSystem: ADCS
 *  Date: 2020-02-13
 *
 *	Description: Contains the function "sun_vector_estimate" which takes the
 *	sun sensor intensity eigen matrix "y", eigen matrix of sun sensor normals
 *	"H", and returns sun vector estimate as an eigen matrix from a body-fixed
 *	frame "s_hat_BF".
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

ret_val SunEstimate(Eigen::MatrixXd intensity_matrix, Eigen::MatrixXd normals_matrix, Eigen::Vector3d  &sun_estimate) {
	/****************************************
	Description: Takes sun sensor intensity eigen matrix, eigen matrix of sun sensor normals,
	and returns sun vector estimate as an eigen matrix from a body-fixed frame.
	
	input: 
		- intensity_matrix

		- normals_matrix

	output: 
		- sun_estimate
	****************************************/
	
	// Calculates eigen matrix of sun sensors above threshold number "sens" and integer "r"
	// of sun sensors above threshold number 0.5.
	Eigen::MatrixXd sens(1, 1);
	int inc = 0;
	for (int j = 0; j < 18; j++) {
		if (intensity_matrix(j, 0) > 0.5) {
			sens.conservativeResize(inc + 1, 1);
			sens(inc, 0) = j + 1;
			inc++;
		}
	}
	int r = sens.rows();

	//If there are less than 3 sensors that can see the Sun, the estimate cannot be determined.
	if (r < 3) {
		sun_estimate << 0,
						0,
						0;
	}
	else {

		//Remaps sun sensor intensity vector "y" to keep only values above intensity threshold.
		Eigen::MatrixXd tempY(sens.rows(), 1);
		for (int i = 0; i < sens.rows(); i++) {
			tempY(i, 0) = intensity_matrix(sens(i, 0) - 1, 0);
		}
		intensity_matrix = tempY;

		/*
		Remaps sun sensor normals vector "H" to keep only normals with sun sensor values
		above intensity threshold.
		*/
		Eigen::MatrixXd tempH(sens.rows(), 3);
		for (int i = 0; i < sens.rows(); i++) {
			for (int j = 0; j < 3; j++) {
				tempH(i, j) = normals_matrix(sens(i, 0) - 1, j);
			}
		}
		normals_matrix = tempH;

		/*
		Checks if reciprical condition of "H"'s conjugate transpose * "H" is well conditioned.
		If it's poorly conditioned, the sun vector estimate cannot be determined.
		*/
		if (rcond(normals_matrix.conjugate().transpose()*normals_matrix) > 1E-6) {
			//Determines sun vector estimate. 
			sun_estimate = (normals_matrix.conjugate().transpose()*normals_matrix)
			.inverse()*normals_matrix.conjugate().transpose()*intensity_matrix;
		}
		else {
			sun_estimate << 0,
							0,
							0;
		}
	}
	
	return SUCCESS;
}
