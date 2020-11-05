#pragma once

#ifndef EXTENDED_KALMAN_FILTER_RW_H
#define EXTENDED_KALMAN_FILTER_RW_H

/* INITIALIZATIONS OF CONSTANTS */
/* sizes of input arrays */
#define y_size_xaxis	10
	/* each row of y corresponds to:
	1. OMEGAx RW angular velocity of x RW relative to BF expressed in
	body-fixed (BF) axes
	2. OMEGAy RW angular velocity of y RW relative to BF expressed in BF
	3. OMEGAz RW angular velocity of z RW relative to BF expressed in BF
	4. wx CubeSat angular velocity relative to Earth-Centered-Inertia (ECI) expressed in BF
	   (x component) rad/s
	5. wy CubeSat angular velocity relative to ECI expressed in BF
	   (y component) rad/s
	6. wz CubeSat angular velocity relative to ECI expressed in BF
	   (z component) rad/s
	7. q1 element 1 of vector portion of quaternion representing rotation from ECI to BF
	8. q2 element 2 of vector portion of quaternion representing rotation from ECI to BF
	9. q3 element 3 of vector portion of quaternion representing rotation from ECI to BF
	10. q0 scalar element quaternion representing rotation from ECI to BF */
#define u_size_xaxis	6
	/* % each row of u corresponds to:
	1. Tx applied external torque x (in BF) Nm
	2. Ty applied external torque y (in BF) Nm
	3. Tz applied external torque z (in BF) Nm
	4. RWx applied RW torque x (in BF) Nm
	5. RWy applied RW torque y (in BF) Nm
	6. RWz applied RW torque z (in BF) Nm */
#define desired_size_xaxis	11
	/* % each column of desired corresponds to:
	1. OMEGAx RW angular velocity of x RW relative to BF expressed in BF
	2. OMEGAy RW angular velocity of y RW relative to BF expressed in BF
	3. OMEGAz RW angular velocity of z RW relative to BF expressed in BF
	4. wx CubeSat angular velocity relative to ECI expressed in BF
	   (x component) rad/s
	5. wy CubeSat angular velocity relative to ECI expressed in BF
	  (y component) rad/s
	6. wz CubeSat angular velocity relative to ECI expressed in BF
	   (z component) rad/s
	7. q1 element 1 of vector portion of quaternion representing rotation from ECI to BF
	8. q2 element 2 of vector portion of quaternion representing rotation from ECI to BF
	9. q3 element 3 of vector portion of quaternion representing rotation from ECI to BF
	10. q0 scalar element quaternion representing rotation from ECI to BF
	11. betax onboard gyro angular rate bias on BF x axis
	12. betay onboard gyro angular rate bias on BF y axis
	13. betaz onboard gyro angular rate bias on BF z axis */
#define input_size_yaxis	251

	/* Cubesat dimensions for model in EKF */
	// assume 2U rectangular prism
#define dim_x	.1	// m
#define dim_y	.1	// m
#define dim_z	.2	// m
#define mass	2	// kg

const double Ixx(1.0 / 12.0 * mass * (pow(dim_y, 2) + pow(dim_z, 2)));	// kgm^2
const double Iyy(1.0 / 12.0 * mass * (pow(dim_x, 2) + pow(dim_z, 2)));	// kgm^2
const double Izz(1.0 / 12.0 * mass * (pow(dim_x, 2) + pow(dim_y, 2)));	// kgm^2

#define tmax	250
#define Ts	1
#define n	13

/* Band-Limited White Gaussian Noise
	variance of noise signal
	standard deviation is sqrt(variance of noise signal)
	68 percent of the generated noise is within plus/minus one standard deviation
	process noise added to equations of motion written in state space
	x_dot = .... + process noise */

	// variance of process noise on OMEGA_dot used to generate test data
#define phi_0_variance  1.0e-1
// variance of process noise on w_dot used to generate test data
#define phi_1_variance	1.0e-6
// variance of process noise on q_dot used to generate test data
#define phi_2_variance	1.0e-5
// variance of process noise on q0_dot used to generate test data
#define phi_3_variance	1.0e-5
// variance of process noise on beta_dot used to generate test data
#define phi_4_variance	1.0e-4

// variance of measurement noise on OMEGA used to generate test data
#define psi_0_variance  1.0e-1
// variance of measurement noise on w used to generate test data
#define psi_1_variance	1.0e-4
// variance of measurement noise on q used to generate test data
#define psi_2_variance	1.0e-3
// variance of measurement noise on q0 used to generate test data
#define psi_3_variance	1.0e-3

#endif //EXTENDED_KALMAN_FILTER_RW_H