#ifndef IGRF12_H
#define IGRF12_H
#include <cmath>
//This file contains the declarations for functions and variables used in the IGRF12 model
//Authored By: James Smith

//legendre function variables
//Inputs
//desired degree to calculate
int mDes;
//desired order to calculate
int nDes;
//cosine of geocentric latitude
double cosTheta;
//sine of geocentric latitude
double sinTheta;

//Class for the legendre function
class legendre {
private:
	//Outputs
	//Schmidt semi-normalized legendre function
	double pMNBar;
	//partial derivative of Schmidt semi-normalized legendre function
	double dpMNBar;
	//function arrays
	double p[];
	double dp[];
	double pMN;
	double dpMN;
	//other function variables
	char delta;
public:
	legendre(int mDes, int nDes, double sinTheta, double cosTheta );
	void setP(int m, int n, double val);
	void setdP(int m, int n, double val);
	void setDelta(int Mdes);
	void calcPMN();
	void calcdPMN();
	void calcdPMNDes();
};


#endif
