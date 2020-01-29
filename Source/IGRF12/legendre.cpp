//This file contains the implementation on how to create the legendre model
//Authored by James Smith

#include "igrf12.hpp"

//factorial function
int factorial(int n){
	if (n==1){
		return 1;
	}
	else{
		return n*factorial(n-1);
	}
}

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
	calcdPmnPmn();
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

void legendre::calcdPmnPmn() {
	//recursive formula based off of Schmidt semi/quasi-normalized Legendre polynomial function
	//formulated by Anna Wailand
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
        dpMN = cosTheta;
	}
    else{
        //initialize matrix
        p(0,0) = 1;
        p(1,0) = cosTheta;
        p(1,1) = sinTheta;
        dp(0,0) = 0;
        dp(1,0) = -sinTheta;
        dp(1,1) = cosTheta;
        //use recursive methose to calculate pMN and dPMN
        for (int n=2; n<nDesired; n++){
            for(int m = 0; m<mDesired; m++){
                if (m==0){
                    p(n,m) = 1/n*((2*n-1)*cosTheta*p(n-1,m)-(n-1)*p(n-2,m));
                    dp(n,m) = 1/n*((2*n-1)*(cosTheta*dp(n-1,m)-sinTheta*p(n-1,m))-(n-1)*dp(n-2,m));
                }
                else if(m == n){
                    p(n,m) = (2*n-1)*sinTheta*p(n-1,m-1);
                    dp(n,m) = (2*n-1)*(sinTheta*dp(n-1,m-1)+cosTheta*p(n-1,m-1));
                }
				else{
					p(n,m) = cosTheta*p(n-1,m)+(n+m-1)*sinTheta*p(n-1,m-1);
					dp(n,m) = cosTheta*dp(n-1,m)-sinTheta*p(n-1,m)+(n+m-1)*(sinTheta*dp(n-1,m-1)+cosTheta*p(n-1,m-1));
				}
            }
        }
		pMN = p(nDesired,mDesired);
		dpMN = dp(nDesired,mDesired);
    }
	//calculate pMNBar
	pMNBar = sqrt((2-delta)*factorial(nDesired-mDesired)/factorial(nDesired+mDesired))*pMN;
	//calculate dPMNBar by division to save recursion
	dpMNBar = pMNBar*dpMN/pMN;
} 