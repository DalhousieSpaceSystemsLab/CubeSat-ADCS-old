#pragma once

#ifndef EXTENDED_KALMAN_FILTER_H
#define EXTENDED_KALMAN_FILTER_H

/* INITIALIZATIONS OF CONSTANTS */
/* sizes of input arrays */
#define y_size_xaxis	7
	/* each row of y corresponds to:
	1. wx CubeSat angular velocity relative to ECI expressed in BF
	   (x component) rad/s
	2. wy CubeSat angular velocity relative to ECI expressed in BF
	   (y component) rad/s
	3. wz CubeSat angular velocity relative to ECI expressed in BF
	   (z component) rad/s
	4. q1 element 1 of vector portion of quaternion representing rotation from ECI to BF
	5. q2 element 2 of vector portion of quaternion representing rotation from ECI to BF
	6. q3 element 3 of vector portion of quaternion representing rotation from ECI to BF
	7. q0 scalar element quaternion representing rotation from ECI to BF*/
#define u_size_xaxis	3
	/* % each row of u corresponds to:
	1. Tx applied torque x (in BF) Nm
	2. Ty applied torque y (in BF) Nm
	3. Tz applied torque z (in BF) Nm */
#define desired_size_xaxis	11
	/* each column of desired corresponds to:
	1. data time stamp, seconds (measurement sample rate is 1 second)
	2. wx CubeSat angular velocity relative to ECI expressed in BF
		  (x component) rad/s
	3. wy CubeSat angular velocity relative to ECI expressed in BF
		  (y component) rad/s
	4. wz CubeSat angular velocity relative to ECI expressed in BF
		  (z component) rad/s
	5. q1 element 1 of vector portion of quaternion representing rotation from ECI to BF
	6. q2 element 2 of vector portion of quaternion representing rotation from ECI to BF
	7. q3 element 3 of vector portion of quaternion representing rotation from ECI to BF
	8. q0 scalar element quaternion representing rotation from ECI to BF
	9. betax onboard gyro angular rate bias on BF x axis
	10. betay onboard gyro angular rate bias on BF y axis
	11. betaz onboard gyro angular rate bias on BF z axis */
#define input_size_yaxis	251

	/* Cubesat dimensions for model in EKF */
	// assume 2U rectangular prism
#define dim_x	.1	// m
#define dim_y	.1	// m
#define dim_z	.2	// m
#define mass	2	// kg
const float Ixx	(1.0/12.0*mass*(pow(dim_y, 2) + pow(dim_z, 2)));	// kgm^2
const float Iyy	(1.0 / 12.0*mass*(pow(dim_x, 2) + pow(dim_z, 2)));	// kgm^2
const float Izz	(1.0 / 12.0*mass*(pow(dim_x, 2) + pow(dim_y, 2)));	// kgm^2
#define tmax	100
#define Ts	1
#define n	10

/* Band-Limited White Gaussian Noise
	variance of noise signal
	standard deviation is sqrt(variance of noise signal)
	68 percent of the generated noise is within plus/minus one standard deviation

	process noise added to equations of motion written in state space
	x_dot = .... + process noise */

	// variance of process noise on w_dot used to generate test data
#define phi_1_variance	1.0e-6
// variance of process noise on q_dot used to generate test data
#define phi_2_variance	1.0e-5
// variance of process noise on q0_dot used to generate test data
#define phi_3_variance	1.0e-5
// variance of process noise on beta_dot used to generate test data
#define phi_4_variance	1.0e-4

// variance of measurement noise on w used to generate test data
#define psi_1_variance	1.0e-4
// variance of measurement noise on q used to generate test data
#define psi_2_variance	1.0e-3
// variance of measurement noise on q0 used to generate test data
#define psi_3_variance	1.0e-3

#endif //EXTENDED_KALMAN_FILTER_H