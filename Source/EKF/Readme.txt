 	Author: Mark MacGillivray (an implementation of Dr. Bauer's algorithm)
 	Project: Dalhousie CubeSat
	SubSystem: ADCS
 	Date: 2020-11-22
 
	Description: Contains the EKF class which can update x_hat_kk and P_kk vectors of the Extended Kalman Filter with asymmetrically positioned reaction wheels using the "update" member function, which is passed the u and y matricies.
  
	Each row of y corresponds to :
	1. ws1 scalar angular speed of RW#1 relative to BF
	2. ws2 scalar angular speed of RW#2 relative to BF
	3. ws3 scalar angular speed of RW#3 relative to BF
	4. wx CubeSat angular velocity relative to ECI expressed in BF % (x component) rad / s
	5. wy CubeSat angular velocity relative to ECI expressed in BF
	(y component) rad / s
	6. wz CubeSat angular velocity relative to ECI expressed in BF
	(z component) rad / s
	7. q1 element 1 of vector portion of quaternion representing rotation from ECI to BF
	8. q2 element 2 of vector portion of quaternion representing rotation from ECI to BF
	9. q3 element 3 of vector portion of quaternion representing rotation from ECI to BF
	10. q0 scalar element quaternion representing rotation from ECI to BF

	Each row of u corresponds to :
	1. gx applied external torque x(in BF) Nm
	2. gy applied external torque y(in BF) Nm
	3. gz applied external torque z(in BF) Nm
	4. ga1 applied scalar torque from motor on RW#1 Nm
	5. ga2 applied scalar torque from motor on RW#2 Nm
	6. ga3 applied scalar torque from motor on RW#3 Nm

	x_hat_kk and P_kk can be accessed by the member functions "get_x_hat_kk" and "get_P_kk".