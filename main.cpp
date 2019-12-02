/*
 * Authors: Cathy Song and...
 * Project: Dalhousie CubeSat
 * SubSystem: ADCS
 * Date:   2019-11-22
 *
 * This file contains the main function for the ADCS module. 
 *
 */


#include "pch.h"
#include <iostream>
#include <fstream>
#include "Eigen/Dense"

#define	testparse	// outputs to terminal for testing the parse function

enum returnvals{FAIL, SUCCESS};
#define IGRF_READLINES	104						// number of linesevery 5 years
#define MAX_MN_VALUE	13+1					// max m and n values, (ignoring 0)



/* storage arrays for IGRF 12 parsing */
/* storage arrays for IGRF 12 parsing */
float g_nominal[MAX_MN_VALUE][MAX_MN_VALUE];	// IGRF g coefficient nanoTesla (nT)
float h_nominal[MAX_MN_VALUE][MAX_MN_VALUE];	// IGRF h coefficient nanoTesla (nT)
float SV_g[MAX_MN_VALUE][MAX_MN_VALUE];		// secular variation of g (nT/year)
float SV_h[MAX_MN_VALUE][MAX_MN_VALUE];		// secular variation of h (nT/year)

using Eigen::Matrix3d;
using namespace std;

int Parse_IGRF(void);

int main()
{
	if (Parse_IGRF()) {
		// continue the program
		cout << "\n\nnow we can continue with the algorithm!";
	}
	else {
		cout << "unable to open the IGRF file\n";	// diagnostic
	}
}

int Parse_IGRF(void) {
	/************************************************************************
	Description: This function parses the IGRF.txt file which contains the
	gmn, hmn, and SVgmn, SVhmn values required for the IGRF algorithm.
	
	Author: Cathy

	input: none
	output: SUCCESS|FAIL diagnostic
	*************************************************************************/
	ifstream IGRFconstants;
	IGRFconstants.open("IGRF12.txt", ifstream::in);

	/* parsing variables */
	int m = 0, n = 0, i = 0;
	string garbage;

	if (!IGRFconstants) {
#ifdef testparse
#endif
		IGRFconstants.close();		// close the file if opening failed
		return FAIL;
	}
	else {
		cout << "opened the IGRF file\n\n";	// file opening was successful
		cout << "n (x axis), m (y axis):\n";
		while (IGRFconstants) {
			IGRFconstants >> n >> m;
			IGRFconstants >> g_nominal[m][n];
			IGRFconstants >> h_nominal[m][n];
			IGRFconstants >> SV_g[m][n];
			IGRFconstants >> SV_h[m][n];
			IGRFconstants >> garbage >> i;

			// interpretation f data was successful
			//cout << n << " " << m << " " << g_nominal[m][n] << " " << h_nominal[m][n] << " " << SV_g[m][n] << " " << SV_h[m][n];
		}
		cout << "g values(m,n): \n";
		for (n = 0; n < MAX_MN_VALUE; n++) {
			for (m = 0; m < MAX_MN_VALUE; m++) {
				cout << g_nominal[m][n] << "\t";
			}
			cout << "\n";
		}
		/* print the h values with respect to m and n*/
		cout << "\nh values(m,n): \n";
		for (n = 0; n < MAX_MN_VALUE; n++) {
			for (m = 0; m < MAX_MN_VALUE; m++) {
				cout << h_nominal[m][n] << "\t";
			}
			cout << "\n";
		}
		cout << "SVg values(m,n): \n";
		for (n = 0; n < MAX_MN_VALUE; n++) {
			for (m = 0; m < MAX_MN_VALUE; m++) {
				cout << SV_g[m][n] << "\t";
			}
			cout << "\n";
		}
		cout << "SVh values(m,n): \n";
		for (n = 0; n < MAX_MN_VALUE; n++) {
			for (m = 0; m < MAX_MN_VALUE; m++) {
				cout << SV_h[m][n] << "\t";
			}
			cout << "\n";
		}
	}
	IGRFconstants.close();	// close the file
	return SUCCESS;
}