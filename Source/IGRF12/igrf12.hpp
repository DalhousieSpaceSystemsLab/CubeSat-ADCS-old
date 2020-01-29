#ifndef IGRF12_H
#define IGRF12_H
#include <cmath>
#include <eigen3/Eigen/Dense>
//This file contains the declarations for functions and variables used in the IGRF12 model
//Authored By: James Smith
using namespace Eigen;
//legendre function variables
//Inputs
//desired degree to calculate
int mDesired;
//desired order to calculate
int nDesired;
//cosine of geocentric latitude
double cosTheta;
//sine of geocentric latitude
double sinTheta;

//factorial function 
int factorial(int n);

//Class for the legendre function
class legendre {
private:
	//Outputs
	//Schmidt semi-normalized legendre function
	double pMNBar;
	//partial derivative of Schmidt semi-normalized legendre function
	double dpMNBar;
	//function arrays
	MatrixXf p;//should be replaced with matrix from eigen
	MatrixXf dp;//should be replaced with matrix from eigen
	double pMN;
	double dpMN;
	//other function variables
	char delta;
public:
	legendre(int mDesired, int nDesired, double sinTheta, double cosTheta );
	void setDelta(int mDesired);
	void calcdPmnPmn();
	void calcdPMNDesired();
};


#endif