#define _USE_MATH_DEFINES
#include "nadir.h"
Eigen::MatrixXd C1(double x) {
	/****************************************
	Places scalar angle x(rad) into standard
	rotation matrix about x - axis.
	input: x
	output: C
	****************************************/

	Eigen::MatrixXd C(3, 3);
	C << 1, 0, 0,
		0, cos(x), sin(x),
		0, -sin(x), cos(x);
	return C;
}

Eigen::MatrixXd C2(double y) {
	/****************************************
	Places scalar angle y(rad) into standard
	rotation matrix about y - axis.
	input: y
	output: C
	****************************************/

	Eigen::MatrixXd C(3, 3);
	C << cos(y), 0, -sin(y),
		0, 1, 0,
		sin(y), 0, cos(y);
	return C;
}

Eigen::MatrixXd C3(double z) {
	/****************************************
	Places scalar angle z(rad) into standard
	rotation matrix about z - axis.
	input: z
	output: C
	****************************************/

	Eigen::MatrixXd C(3, 3);
	C << cos(z), sin(z), 0,
		-sin(z), cos(z), 0,
		0, 0, 1;
	return C;
}

Eigen::MatrixXd set_C_PI(double omega, double RAAN, double i) {
	/****************************************
	ECI to Perifocal rotation matrix, making use of orbital elements:
	omega = argument of perigee(rad)
	i = inclination(rad)
	RAAN = right ascension of the ascending node(rad)
	input: omega, i, RAAN
	output: C_PI
	****************************************/

	Eigen::MatrixXd C_PI;
	C_PI = C3(omega) * C1(i) * C3(RAAN);
	return C_PI;
}

Eigen::MatrixXd set_C_NPP(double tano) {
	/****************************************
	Perifocal to NP. Both frames lie in the orbital plane, perifocal is an
	inertial coordinate system while NP follows the satellite. The rotation
	is a 3 - 2 - 3 sequence through true anomaly, -90, and 90, respectively.
	tano = true anomaly(rad).
	input: tano
	output: C_NPP
	****************************************/

	Eigen::MatrixXd C_NPP;
	C_NPP = C3(M_PI / 2) * C2(-M_PI / 2) * C3(tano);
	return C_NPP;
}

Eigen::MatrixXd set_qq(Eigen::MatrixXd C_NPI) {
	/****************************************
	Sets qq matrix, the first pass of nadir quaternion coordinates.
	input: C_NPI
	output: qq
	****************************************/

	Eigen::MatrixXd qq(4, 1), t(3, 3);
	t = C_NPI;

	//Calculates first pass of quaternion.
	qq << sqrt(0.25 * (1 + t(0, 0) + t(1, 1) + t(2, 2))),
		sqrt(0.25 * (1 + t(0, 0) - t(1, 1) - t(2, 2))),
		sqrt(0.25 * (1 - t(0, 0) + t(1, 1) - t(2, 2))),
		sqrt(0.25 * (1 - t(0, 0) - t(1, 1) + t(2, 2)));
	return qq;
}
Eigen::MatrixXd solve_q_NPI(Eigen::MatrixXd C_NPI) {
	/****************************************
	Takes the combination of rotation matricies, C_NPI, and
	returns quaternion for nadir.
	input: C_NPI
	output: q_NPI
	****************************************/

	int idx = 0;
	double qs, qx, qy, qz;
	Eigen::MatrixXd q_NPI(1, 4), t(3, 3), qq;

	t = C_NPI;

	//Sets first pass of quaternion.
	qq = set_qq(C_NPI);

	//Finds index of maximum row element of qq matrix.
	for (int i = 0; i < qq.rows(); i++) {
		if (qq(i, 0) == qq.maxCoeff()) {
			idx = i;
		}
	}

	//Constructs quaternion components based on index.
	if (idx == 0) { // qs is max
		qs = qq(0, 0); // original
		qx = (t(2, 1) - t(1, 2)) / (4 * qs);
		qy = (t(0, 2) - t(2, 0)) / (4 * qs);
		qz = (t(1, 0) - t(0, 1)) / (4 * qs);
	}
	else if (idx == 1) { // qx is max
		qx = qq(1, 0);  // original
		qs = (t(2, 1) - t(1, 2)) / (4 * qx);
		qy = (t(1, 0) + t(0, 1)) / (4 * qx);
		qz = (t(0, 2) + t(2, 0)) / (4 * qx);
	}
	else if (idx == 2) { // qy is max
		qy = qq(2, 0);  // original
		qs = (t(0, 2) - t(2, 0)) / (4 * qy);
		qx = (t(1, 0) + t(0, 1)) / (4 * qy);
		qz = (t(2, 1) + t(1, 2)) / (4 * qy);
	}
	else if (idx == 3) {  //qz is max
		qz = qq(3, 0); // original
		qs = (t(1, 0) - t(0, 1)) / (4 * qz);
		qx = (t(0, 2) + t(2, 0)) / (4 * qz);
		qy = (t(2, 1) + t(1, 2)) / (4 * qz);
	}

	//Values for the final quaternion are initialized.
	q_NPI << qs, qx, qy, qz;

	return q_NPI;
}

Eigen::MatrixXd nadir(
	/****************************************
	Takes node longitude(omega), right ascension of the
	ascending node (RAAN), inclination (i),true anomaly
	(tano) in degrees, and returns the nadir quaternion as
	a 1x4 matrix (q_NPI):
	input: omega, RAAN, i, tano
	output: q_NPI
	****************************************/

	double omega_deg,
	double RAAN_deg,
	double i_deg,
	double tano_deg)
{
	Eigen::MatrixXd C_PI, C_NPP, C_NPI, qq(4, 1), t, q_NPI(1, 4);
	double d2r, omega, RAAN, i, tano;

	//Conversion from degrees to radians.
	d2r = M_PI / 180;
	omega = d2r * omega_deg;
	RAAN = d2r * RAAN_deg;
	i = d2r * i_deg;
	tano = d2r * tano_deg;

	//Finds rotation matricies.
	C_PI = set_C_PI(omega, RAAN, i);
	C_NPP = set_C_NPP(tano);

	//Combines both rotation matricies.
	C_NPI = C_NPP * C_PI;

	//Finds nadir quaternion.
	q_NPI = solve_q_NPI(C_NPI);

	return q_NPI;
}
