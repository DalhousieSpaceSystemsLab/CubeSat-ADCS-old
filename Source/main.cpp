// ADCS.cpp : This file contains the 'main' function. Program execution begins and ends there.
//
//#include "adcsDataPackages.hpp"
//#include "DPP.h"
#include "Extended_Kalman_Filter_RW.h"
#include "igrf.h"
#include "Nadir.h"
#include "Q_Method.h"
#include "Sun_Vector_Estimate.h"
#include "Eigen/Dense"
#include "PDcontroller.h"
#include "Bdot.h"

//Temporary files used for testing.
#include "main_test.h"
#include "main_test.cpp"
#include "data.h"
#include <iostream>
#include <string>

/*
To get a better understanding of this code, you can take a look at slides 3, 9, and 10 in the ADCS CDR presentation as well as see the high level system diagram.
The code currently doesn't support any of the other modes of operation. Its current form resembles "standby mode", which calculates voltages to be sent to the
reaction wheels based on sensor data provided by the MSP430 and GPS data provided by the GPS server. Because there is no UART or IPC functionality at the moment,
example GPS and sensor data provided by "main_test.h" are used to demonstrate how the core architecture works.
*/

void leop();
void safehold();
void standby();
void faultDetection();
void detumbleAndDesaturate();
void updateEkf(Eigen::Vector4d attitudeEstimate, EKF &ekf);
void bodyRateCheck(const Eigen::Matrix<double, 12, 1>& rawAngRate, double &bodyRateNorm, double &rwRateNormRPM);
void sunVecAvailabilityCheck(const Eigen::Matrix<double, 18, 1>& y, bool &sunVecAvailable, int &secondsWithoutSun);
void useEkfPredictedState();
void useReactionWheels(Eigen::Vector3d w_est, Eigen::Vector4d q_est, Eigen::Vector4d q_NPI, EKF ekf, Eigen::Vector3d &reactionWheelVoltages);
void zeroEkfState(EKF &ekf);
Eigen::Vector3d getAngVelFromEkf(EKF ekf);
Eigen::Vector4d getQuatFromEkf(EKF ekf);

int main()
{  
    std::string modeOfOperation = "standby";
    
    
    
    if(modeOfOperation == "standby"){
        standby();  
    }else if(modeOfOperation == "safehold"){
        safehold();
    }
}

//Modes of operation (slide 3)

void leop(){
}

void safehold(){
}

void faultDetection(){
}

//Controller logic flow functions to be implemented for standby mode (see slide 10 on ADCS CDR presentation)

void detumbleAndDesaturate(){
}

Eigen::Vector3d getAngVelFromEkf(EKF ekf){
    Eigen::Vector3d w_est;
    w_est[0] = ekf.get_x_hat_kk()(3);
    w_est[1] = ekf.get_x_hat_kk()(4);
    w_est[2] = ekf.get_x_hat_kk()(5);
    return w_est;
}

Eigen::Vector4d getQuatFromEkf(EKF ekf){
    Eigen::Vector4d q_est;
    q_est[0] = ekf.get_x_hat_kk()(9); // scalar was to the right of vector components in EKF.
    q_est[1] = ekf.get_x_hat_kk()(6);
    q_est[2] = ekf.get_x_hat_kk()(7);
    q_est[3] = ekf.get_x_hat_kk()(8);
    return q_est;
}

void updateEkf(Eigen::Vector4d attitudeEstimate, EKF &ekf){
    Eigen::Matrix<double, 6, 1> u = Eigen::Matrix<double, 6, 1>::Zero();
    Eigen::Matrix<double, 10, 1> y1 = Eigen::Matrix<double, 10, 1>::Zero();
    
    y1[0] = s.rawAngRate(0);        //ws1 scalar angular speed of RW#1 relative to BF
    y1[1] = s.rawAngRate(1);        //ws2 scalar angular speed of RW#2 relative to BF
    y1[2] = s.rawAngRate(2);        //ws3 scalar angular speed of RW#3 relative to BF
    y1[3] = s.rawAngRate(3);        //wx CubeSat angular velocity relative to ECI expressed in BF (x component) rad/s
    y1[4] = s.rawAngRate(4);        //wy CubeSat angular velocity relative to ECI expressed in BF (y component) rad/s
    y1[5] = s.rawAngRate(5);        //wz CubeSat angular velocity relative to ECI expressed in BF (z component) rad/s
    y1[6] = attitudeEstimate(1);    //q1 element 1 of vector portion of quaternion representing rotation from ECI to BF
    y1[7] = attitudeEstimate(2);    //q2 element 2 of vector portion of quaternion representing rotation from ECI to BF
    y1[8] = attitudeEstimate(3);    //q3 element 3 of vector portion of quaternion representing rotation from ECI to BF
    y1[9] = attitudeEstimate(0);    //q0 scalar element quaternion representing rotation from ECI to BF

    u[0] = s.rawAngRate(6);         //gx applied external torque x (in BF) Nm
    u[1] = s.rawAngRate(7);         //gy applied external torque y (in BF) Nm
    u[2] = s.rawAngRate(8);         //gz applied external torque z (in BF) Nm
    u[3] = s.rawAngRate(9);         //ga1 applied scalar torque from motor on RW#1 Nm
    u[4] = s.rawAngRate(10);        //ga2 applied scalar torque from motor on RW#2 Nm
    u[5] = s.rawAngRate(11);        //ga3 applied scalar torque from motor on RW#3 Nm
    
    ekf.update(u, y1);
}
void bodyRateCheck(const Eigen::Matrix<double, 12, 1>& rawAngRate, double &bodyRateNorm, double &rwRateNormRPM){
Eigen::Vector3d bodyRate, rwRate;
bodyRate(0) = rawAngRate(3);
bodyRate(1) = rawAngRate(4);
bodyRate(2) = rawAngRate(5);

bodyRateNorm = bodyRate.norm();

rwRate(0) = rawAngRate(0);
rwRate(1) = rawAngRate(1);
rwRate(2) = rawAngRate(2);

rwRateNormRPM = rwRate.norm()*60/(2*M_PI);
}
void sunVecAvailabilityCheck(const Eigen::Matrix<double, 18, 1>& y, bool &sunVecAvailable, int &secondsWithoutSun){
    /*
    If more than 15 of the 18 sun sensors are returning values under the threshold of 0.5, the LORIS is considered to be in an eclipse.
    this function returns true if in eclipse, and false if not in eclipse.
    */
   int darkenedSensors = 0;
    for (int i = 0; i < 18; i++){
        if (y(i) < 0.5){
            darkenedSensors++;
        }
    }
    if (darkenedSensors > 15){
        sunVecAvailable = false;
        secondsWithoutSun++;
    }else{
        sunVecAvailable = true;
        secondsWithoutSun = 0;
    }
}

void useReactionWheels(Eigen::Vector3d w_est, Eigen::Vector4d q_est, Eigen::Vector4d q_NPI, EKF ekf, Eigen::Vector3d &reactionWheelVoltages){
            w_est = getAngVelFromEkf(ekf);
            q_est = getQuatFromEkf(ekf);
            reactionWheelVoltages = pd(w_est, q_est, q_NPI);
}

void zeroEkfState(EKF &ekf){
    ekf.set_x_hat_kk(Eigen::Matrix<double, 13, 1>::Zero());
    ekf.set_P_kk(Eigen::Matrix<double, 13, 13>::Zero());

}
void standby(){
bool sunVecAvailable = false;
    bool desaturate = false;
    int secondsWithoutSun;
    double bodyRateNorm;
    double rwRateNormRPM;
    
    //Initializing the EKF with the initial values provided by Dr. Bauer's Matlab code.
    Eigen::Matrix<double, 13, 1> x_hat_kk_initial = Eigen::Matrix<double, 13, 1>::Zero();
    x_hat_kk_initial << -.02, .04, .01, -.01, .01, .002, .5, -.25, .3, .8, .1, -.1, .15;
    
     
    //EKF requires some persistent variables, so it is treated as a class instead of a function like the other modules.
    EKF ekf(x_hat_kk_initial); 
   
    /*The nadir and sun vector estimate modules can be initialized at the beginning because they are passed variables directly from
    the GPS and sensors.
    */
    Eigen::MatrixXd q_NPI = nadir(g.omega, g.RAAN, g.i, g.tano);
    Eigen::MatrixXd estSunVec = sun_vector_estimate(s.rssd.y, s.rssd.H);
    
    //Magnetic and sun vector values are found in order to be passed to the q method.
    Eigen::Vector3d b1 = estSunVec;
    Eigen::Vector3d b2 = s.rmfvA;
    Eigen::Vector3d b3 = s.rmfvB;

    //***********************************
    //r1 will be calculated when sun vector reference code is merged. For now these temporary values will be used.
    Eigen::Vector3d r1;
    r1[0] = 0.577350269189626,
    r1[1] = 0.577350269189626,
    r1[2] = -0.577350269189626;
    //***********************************
    
    //This is the reference magnetic field vector.
    Eigen::Vector3d r2;
    MagReference(g.lat_geodetic, g.phi, g.H, g.year, g.month, g.day, r2);

    //These values are passed to the q method and the attitude estimate quaternion is returned.
    Eigen::Vector4d attitudeEstimate = q_method(b1, b2, b3, r1, r2);
    
    Eigen::Vector4d q_NPI_transpose = q_NPI.transpose();// Nadir algorithm needs to return 4x1 instead of 1x4.
    Eigen::Vector3d reactionWheelVoltages;
    Eigen::Vector3d w_est;
    Eigen::Vector4d q_est;

    bodyRateCheck(s.rawAngRate, bodyRateNorm, rwRateNormRPM);
    sunVecAvailabilityCheck(s.rssd.y, sunVecAvailable, secondsWithoutSun);

if(bodyRateNorm > 0.04 || rwRateNormRPM > 5000.0){
    desaturate = true;
    reactionWheelVoltages = Eigen::Vector3d::Zero(); //Do RW voltages need to be zeroed?
    //use Bdot to calculate magnetic torques for magnetorquers (detumble).
}else{
  if(sunVecAvailable){ 
        updateEkf(attitudeEstimate, ekf);               
        useReactionWheels(w_est, q_est, q_NPI_transpose, ekf, reactionWheelVoltages);

    }else{
        if(secondsWithoutSun < 300.0){
            //Not updating ekf, using its last predicted state.    
            useReactionWheels(w_est, q_est, q_NPI_transpose, ekf, reactionWheelVoltages);
        }else{
            zeroEkfState(ekf);
            reactionWheelVoltages = Eigen::Vector3d::Zero(); //Do RW voltages need to be zeroed?
            //use Bdot to calculate magnetic torques for magnetorquers (detumble).
        }        
    }  
}
    std::cout << reactionWheelVoltages; //Printing the voltages in BF to go to MSP430.
}