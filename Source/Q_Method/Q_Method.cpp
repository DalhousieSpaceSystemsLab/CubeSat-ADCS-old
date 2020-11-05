#include "Q_Method.h"

Eigen::MatrixXd B_solve(
	Eigen::MatrixXd b1,
	Eigen::MatrixXd b2,
	Eigen::MatrixXd b3,
	Eigen::MatrixXd r1,
	Eigen::MatrixXd r2) {
	/****************************************
	Takes components of sun vectors and magnetic field vectors
	and returns B matrix using weights w1, w2, and w3.
	input: b1, b2, b3, r1, r2
	output: B
	****************************************/

	int w1 = 1, w2 = 1, w3 = 1;
	Eigen::MatrixXd B;
	B = w1 * (b1 * r1.conjugate().transpose()) + w2 * (b2 *
		r2.conjugate().transpose()) + w3 * (b3 *
		r2.conjugate().transpose());
	return B;
}

Eigen::MatrixXd S_solve(Eigen::MatrixXd B) {
	/****************************************
	Takes B matrix and returns S matrix.
	input: B
	output: S
	****************************************/

	Eigen::MatrixXd S;
	S = B + B.conjugate().transpose();
	return S;
}

double sigma_solve(Eigen::MatrixXd B) {
	/****************************************
	Takes B matrix and returns sigma coefficient.
	input: B
	output: sigma
	****************************************/

	double sigma = B.trace();
	return sigma;
}
Eigen::MatrixXd Z_solve(Eigen::MatrixXd B) {
	/****************************************
	Takes B matrix and returns Z matrix.
	input: B
	output: Z
	****************************************/

	Eigen::MatrixXd Z(3, 1), Z_initialize(1, 3);
	Z_initialize << B(1, 2) - B(2, 1), B(2, 0) - B(0, 2), B(0, 1) - B(1, 0);
	Z = Z_initialize.conjugate().transpose();
	return Z;
}

Eigen::MatrixXd K_solve(
	Eigen::MatrixXd b1,
	Eigen::MatrixXd b2,
	Eigen::MatrixXd b3,
	Eigen::MatrixXd r1,
	Eigen::MatrixXd r2) {
	/****************************************
	Takes components of sun vectors and magnetic field vectors
	and returns K matrix.
	input: b1, b2, b3, r1, r2
	output: K
	****************************************/

	double sigma;
	Eigen::MatrixXd K(4, 4), B, S, Z;
	B = B_solve(b1, b2, b3, r1, r2);
	S = S_solve(B);
	sigma = sigma_solve(B);
	Z = Z_solve(B);

	/*
	   * K is a 4x4 matrix of the form:
	   * | S-σI   Z|
	   * |	       |
	   * | ZT     σ|
	   */
	   //Initializes S-σI section of K matrix.
	K(0, 0) = S(0, 0) - sigma * 1;
	K(0, 1) = S(0, 1) - sigma * 0;
	K(0, 2) = S(0, 2) - sigma * 0;
	K(1, 0) = S(1, 0) - sigma * 0;
	K(1, 1) = S(1, 1) - sigma * 1;
	K(1, 2) = S(1, 2) - sigma * 0;
	K(2, 0) = S(2, 0) - sigma * 0;
	K(2, 1) = S(2, 1) - sigma * 0;
	K(2, 2) = S(2, 2) - sigma * 1;

	//Initializes Z section of K matrix.
	K(0, 3) = Z(0, 0);
	K(1, 3) = Z(1, 0);
	K(2, 3) = Z(2, 0);

	//Initializes Z conjugate transpose section of K matrix.
	K(3, 0) = Z.conjugate().transpose()(0, 0);
	K(3, 1) = Z.conjugate().transpose()(0, 1);
	K(3, 2) = Z.conjugate().transpose()(0, 2);

	//Initializes σ section of K matrix.
	K(3, 3) = sigma;
	return K;
}

Eigen::MatrixXd quaternion_estimate(Eigen::MatrixXd K) {
	/****************************************
	Takes K matrix and returns quaternion estimate matrix.
	input: B
	output: q_est
	****************************************/

	int index;
	Eigen::MatrixXd V, D, q_est;
	Eigen::EigenSolver<Eigen::MatrixXd> eigenSolver(K);

	//Initializes V, a matrix with columns of eigenvectors from K matrix.
	V = (eigenSolver.eigenvectors()).real();

	//Initializes D, a matrix of eigenvalues from K matrix.
	D = (eigenSolver.eigenvalues()).real();

	//Finds the index of the maximum eigenvalue.
	for (int i = 0; i < D.rows(); i++) {
		if (D(i, 0) == D.maxCoeff()) {
			index = i;
		}
	}

	//Sets quaternion estimate to be the eigenvector corresponding to the index.
	q_est = V.col(index);
	return q_est;
}

Eigen::MatrixXd q_method(
	Eigen::MatrixXd b1,
	Eigen::MatrixXd b2,
	Eigen::MatrixXd b3,
	Eigen::MatrixXd r1,
	Eigen::MatrixXd r2) {
	/****************************************
	Takes components of sun vectors and magnetic field vectors
	and returns quaternion estimate matrix.
	input: b1, b2, b3, r1, r2
	output: q_est
	****************************************/

	Eigen::MatrixXd K, q_est;
	K = K_solve(b1, b2, b3, r1, r2);
	q_est = quaternion_estimate(K);
	return q_est;
}