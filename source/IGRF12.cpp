//This file contains the implementation on how to create the IGRF12 model
//Authored by James Smith

#include ".../resource/IGRF12.h"

//constructor
legendre::legendre(int mDes, int nDes, double sinTheta, double cosTheta) {
	if (nDes < 1 || mDes < 1) {
		return;
	}
	//create delta
	setDelta(mDes);
	//create the proper sized p and dp
	p = new double[((mDes + 1)*(nDes + 1))];
	dp = new double[((mDes + 1)*(nDes + 1))];
	//calculate values
	calcPMN();
	calcdPMN();
	calcdPMNDes();
}

void legendre::setDelta(int mDes) {
	if (mDes == 0) {
		delta = 1;
	}
	else {
		delta = 0;
	}
	return;
}

void legendre::setP(int n, int m, double val) {
	p[((m*nDes) + n)] = val;
	return;
}

void legendre::setdP(int n, int m, double val) {
	dp[((m*nDes) + n)] = val;
	return;
}

void legendre::calcPMN() {
	if (nDes == 0 && mDes == 0) {
		pMN = 1;
		dpMN = 1;
	}
	else if (nDes == 0 && mDes == 1) {
		pMN = cosTheta;
		dpMN = -sinTheta;
	}
	else if (nDes == 1 && mDses == 0) {
		pMN = sinTheta;

	}
}