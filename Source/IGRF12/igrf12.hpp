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
};

/***************************************************************************/
/* IGRF parser data structures, constants, and */
/***************************************************************************/

enum returnvals{FAIL, SUCCESS};
#define IGRF_READLINES	104						// number of linesevery 5 years
#define MAX_MN_VALUE	13+1					// max m and n values, (ignoring 0)

extern int Parse_IGRF(void);
extern float g_nominal[MAX_MN_VALUE][MAX_MN_VALUE];	// IGRF g coefficient nanoTesla (nT)
extern float h_nominal[MAX_MN_VALUE][MAX_MN_VALUE];	// IGRF h coefficient nanoTesla (nT)
extern float SV_g[MAX_MN_VALUE][MAX_MN_VALUE];		// secular variation of g (nT/year)
extern float SV_h[MAX_MN_VALUE][MAX_MN_VALUE];		// secular variation of h (nT/year)

/* outputs to terminal: block out define to disable outputs */
#define	testparse		// outputs to terminal for testing the parse function

enum returnvals{FAIL, SUCCESS};

#endif