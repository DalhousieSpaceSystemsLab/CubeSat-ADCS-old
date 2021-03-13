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
#include "Timer.h"

//Temporary files used for testing.
#include "main_test.h"
#include "main_test.cpp"
#include "data.h"
#include <iostream>
#include <string>
#include <thread>

/*
To get a better understanding of this code, you can take a look at slides 3, 9, and 10 in the ADCS CDR presentation as well as see the high level system diagram.
The code currently doesn't support any of the other modes of operation. Its current form resembles "standby mode", which calculates voltages to be sent to the
reaction wheels based on sensor data provided by the MSP430 and GPS data provided by the GPS server. Because there is no UART or IPC functionality at the moment,
example GPS and sensor data provided by "main_test.h" are used to demonstrate how the core architecture works.
*/

void leop(std::string &modeOfOperation, int &leopSecondsOfDetumbling, Eigen::Vector4d &q_est);
void safehold();
void standby(Eigen::Vector4d &q_est);
void faultDetection();
void detumbleAndDesaturate();
void updateEkf(Eigen::Vector4d attitudeEstimate, EKF &ekf);
void bodyRateCheck(const Eigen::Matrix<double, 12, 1>& rawAngRate, double &bodyRateNorm, double &rwRateNormRPM);
void sunVecAvailabilityCheck(const Eigen::Matrix<double, 18, 1>& y, bool &sunVecAvailable, int &secondsWithoutSun);
void useEkfPredictedState();
void useReactionWheels(Eigen::Vector3d w_est, Eigen::Vector4d &q_est, Eigen::Vector4d q_NPI, EKF ekf, Eigen::Vector3d &reactionWheelVoltages);
void zeroEkfState(EKF &ekf);
int statusCheck(std::string modeOfOperation);
Eigen::Vector3d getAngVelFromEkf(EKF ekf);
Eigen::Vector4d getQuatFromEkf(EKF ekf);
void main_loop();

//Initializing variables.
int leopSecondsOfDetumbling = 0;
std::string modeOfOperation = "standby";
int status = 0;
Eigen::Vector4d q_est = Eigen::Vector4d::Zero();
uint32_t loop_interval = 1;
Timer *timer;

//Temp variable
const uint32_t main_loop_time = 10;     // For how many seconds the main loop should run

int main()
{  
    // initiate timer object with callback function, interval and debug on

    std::cout << std::this_thread::get_id() << std::endl;
    timer = new Timer(main_loop, loop_interval, true);
    
    timer->start_timer();
    std::this_thread::sleep_for(std::chrono::seconds(main_loop_time));
    timer->stop_timer();

    return 0;
}

void main_loop() {
    //main loop starts here*************
    std::cout << std::this_thread::get_id() << std::endl;
    std::cout << "Loop count = " << timer->get_loop_count() << std::endl;

    std::string cmd = "sb";//checking for commands.
    

    if(cmd == "sh"){//No matter what the current mode of operation is, switch to safehold mode.
        modeOfOperation = "safehold";
    }else if(cmd == "sb"){// If the
        if(modeOfOperation == "safehold"){
            modeOfOperation = "standby";
        }else{
            std::cout << "must be in safehold to use cmd to go to standby."; 
        }  
    }else if(cmd == "status"){
            status = statusCheck(modeOfOperation);
    }else if(cmd == "dir_ok"){
        if(modeOfOperation == "standby"){
            //get quaternion error in BF/NP from PD controller and if it's within a certain threshold, return true or false.
        }else{
            std::cout << "Must be in standby mode.";
        }
    }else if(cmd == "cur_ori"){
        //get quaternion error in BF/NP from PD controller.
    }else{
        std::cout << "not a command.";
    }

    //Modes of operation (slide 3)
    if(modeOfOperation == "leop"){
        leop(modeOfOperation, leopSecondsOfDetumbling, q_est);
    }else if(modeOfOperation == "standby"){
        standby(q_est);  
    }else if(modeOfOperation == "safehold"){
        safehold();
    }else if(modeOfOperation == "faultDetection"){
        faultDetection();
    }
    //main loop ends here*************
}

void leop(std::string &modeOfOperation, int &leopSecondsOfDetumbling, Eigen::Vector4d &q_est){
    double bodyRateNorm, rwRateNormRPM;
    bodyRateCheck(s.rawAngRate, bodyRateNorm, rwRateNormRPM);
    
    if(bodyRateNorm > 0.04 || rwRateNormRPM > 5000.0){
    //detumble
        if (leopSecondsOfDetumbling > 172800){
            modeOfOperation = "faultDetection";
        }
        leopSecondsOfDetumbling++;
    }else{
        modeOfOperation = "standby";
        standby(q_est);
    }
}

void safehold(){
    //ADCS is off.
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

void useReactionWheels(Eigen::Vector3d w_est, Eigen::Vector4d &q_est, Eigen::Vector4d q_NPI, EKF ekf, Eigen::Vector3d &reactionWheelVoltages){
            w_est = getAngVelFromEkf(ekf);
            q_est = getQuatFromEkf(ekf);
            reactionWheelVoltages = pd(w_est, q_est, q_NPI);
}

void zeroEkfState(EKF &ekf){
    ekf.set_x_hat_kk(Eigen::Matrix<double, 13, 1>::Zero());
    ekf.set_P_kk(Eigen::Matrix<double, 13, 13>::Zero());

}
void standby(Eigen::Vector4d &q_est){
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
    Eigen::Vector3d estSunVec(3);
    estSunVec << 0, 0, 0;
    Eigen::Vector3d& sest = estSunVec;
    ret_val x = SunEstimate(s.rssd.y, s.rssd.H, sest);
    
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
    //Eigen::Vector4d q_est;

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
int statusCheck(std::string modeOfOperation){
    int status = 0;
    if(modeOfOperation == "leop"){
        status = 1;
    }else if(modeOfOperation == "standby"){
        status = 2;
    }else if(modeOfOperation == "safehold"){
        status = 3;
    }else if(modeOfOperation == "faultDetection"){
        status = 4;
    }else{
        status = 0; //status undetermined.
    }
    return status;
}