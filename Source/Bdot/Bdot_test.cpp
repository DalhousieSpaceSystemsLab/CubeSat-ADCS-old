/*
 *		Author: Sean Smith
 *		Project: Dalhousie CubeSat
 *		SubSystem: ADCS
 *		Date: 2020-12-28
 *
 * Description: Tests the Bdot algorithm by using defined inputs identical to the ones used
 *		in the simulink model (i.e. manually found from the simulink model using scope blocks). The inputs
 *		are listed as follows:
 *		B - current noisy magentic field reading from IGRF -12 model
 *		B_1 - previous (1 second time step assumption) noisy magentic field reading from IGRF -12 model
 *		fBdot_1 - previous filtered derivative of field (from feedback).
 * 
 *		The current magnatic moment (m) is calculated and can be compared with the simulink model calculation.
 *		I provided the simulink model values in the code for comparison (manually found). 
 *		It is assumed the time step is 1 second and the current test time is at 2 seonds while the previous data time is 1 second.
 * 
 *		To be used with "Bdot.h" and "Bdot.cpp".
 *
 */
#include "Bdot.h"
#include <iostream>


using namespace Eigen;



int main(void) {
	
	RowVectorXd B(3);
	B << 0.000008456, -0.0000009043, 0.00004029;
	RowVectorXd B_1(3);
	B_1 << 0.00001135, 0.000001993, 0.00004319;
	RowVectorXd fBdot_1(3);
	fBdot_1 << 0.0000003343, 0.00000006190, 0.000001261;

	RowVectorXd m = Bdot(B, B_1, fBdot_1);

	std::cout << "m = "<< m;
	std::cout << "\n\nThe magnetic moment (m) calculated from the simulink model at t = 2 seconds is: \nm = (-1.661e-02,1.881e-03,-7.951e-02)\n ";

	return 0;
}



