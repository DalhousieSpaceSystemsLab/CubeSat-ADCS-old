//This file contains the implementation on how to create the IGRF12 model
//Authored by James Smith

#include "IGRF12.h"

//constructor
legendre::legendre(int mDes, int nDes, double sinTheta, double cosTheta) {
	if (nDesired < 1 || mDesired < 1) {
		return;
	}
	//create delta
	setDelta(mDes);
	//create the proper sized p and dp
	p = new double[((mDesired + 1)*(nDesired + 1))];
	dp = new double[((mDesired + 1)*(nDesired + 1))];
	//calculate values
	calcPMN();
	calcdPMN();
	calcdPMNDes();
}

void legendre::setDelta(int mDes) {
	if (mDesired == 0) {
		delta = 1;
	}
	else {
		delta = 0;
	}
	return;
}

void legendre::setP(int n, int m, double val) {
	p[((m*nDesired) + n)] = val;
	return;
}

void legendre::setdP(int n, int m, double val) {
	dp[((m*nDesired) + n)] = val;
	return;
}

void legendre::calcPMN() {
	if (nDesired == 0 && mDesired == 0) {
		pMN = 1;
		dpMN = 1;
	}
	else if (nDesired == 0 && mDesired == 1) {
		pMN = cosTheta;
		dpMN = -sinTheta;
	}
	else if (nDesired == 1 && mDsesired == 0) {
		pMN = sinTheta;

	}
}