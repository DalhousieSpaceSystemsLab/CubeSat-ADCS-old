#include "Bdot.h"

RowVectorXd Bdot(
	RowVectorXd B,
	RowVectorXd B_1,
	RowVectorXd fBdot_1
	) {

	/********************************
	
    The Bdot function is comprised of three parts:

    1. The simple derivative of the magnetic field B, simply the difference
       between the current field reading B and the previous field reading B_1.
       This derivative assumes a sample time of 1 second, field in Tesla

     INPUTS:  B - current noisy magnetic field reading from magnetometer.
              B_1 - previous field reading
     OUTPUTS: Bdot - current derivative of field

    **********************************/
    RowVector3d Bdot = (B - B_1);

    /**********************************
    
    2. The derivative is passed through a simple low-pass (IIR) filter
       The smoothing factor alpha is a hardcoded constant.
     INPUTS:  Bdot - current derivative of field.
              fBdot_1 - previous filtered derivative of field.
     OUTPUTS: fBdot - current filtered derivative of field.

    ***********************************/
    double alpha = 0.03;
    RowVector3d fBdot = alpha*Bdot + (1-alpha)*fBdot_1;

    /*********************************
    
    3. The magnetic moment is obtained from this filtered derivative,
        m = -K*fBdot. This is the value that drives the magnetorquer (solve m = NIS
        where I is the current needed to produce m).
        The B-dot gain K is a hardcoded constant.
     INPUTS:  fBdot - current filtered derivative of field.
     OUTPUTS: m - required current magnetic moment.
        The magnetic moment interacts with the local field to produce control
        torques for the satellite.

     ********************************/
    double K = 70000;
    RowVector3d m = -K*fBdot;
    // m is subject to hardware constraints (�1.4 Am^2) which are not hardcoded
    // into the function.
	
    return m;
}