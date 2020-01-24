//This file contains the implementation on how to create the legendre model
//Authored by James Smith

#include "igrf12.hpp"

//constructor
legendre::legendre(int mDesired, int nDesired, double sinTheta, double cosTheta) {
	if (nDesired < 1 || mDesired < 1) {
		return;
	}
	//create delta
	setDelta(mDesired);
	//create the proper sized p and dp
	p.resize(nDesired,mDesired);
	dp.resize(nDesired, mDesired);
	//calculate values
	calcPMN();
	calcdPMN();
	calcdPMNDesired();
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

void legendre::calcPMN() {
	if (nDesired == 0 && mDesired == 0) {
		pMN = 1;
		dpMN = 1;
	}
	else if (nDesired == 0 && mDesired == 1) {
		pMN = cosTheta;
		dpMN = -sinTheta;
	}
	else if (nDesired == 1 && mDesired == 0) {
		pMN = sinTheta;

	}
} 