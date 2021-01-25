// ADCS.cpp : This file contains the 'main' function. Program execution begins and ends there.
//
//#include "adcsDataPackages.hpp"
#include "Extended_Kalman_Filter_RW.h"
//#include "DPP.h"
#include "igrf.h"
#include "Nadir.h"
#include "Q_Method.h"
#include "Sun_Vector_Estimate.h"
#include <iostream>
#include "Eigen/Dense"
#include "main_test.h"
#include "main_test.cpp"
#include "data.h"

int main()
{   
    //Initializing
    Eigen::Matrix<double, 13, 1> x_hat_kk_initial = Eigen::Matrix<double, 13, 1>::Zero();
    x_hat_kk_initial << -.02, .04, .01, -.01, .01, .002, .5, -.25, .3, .8, .1, -.1, .15;
    EKF ekf(x_hat_kk_initial);
    //struct gpsData g = { 2020, 2020, 7, 18, 51.6413, 231.7821, 257.8729, 30.0 };
    std::cout << g.day;
    


























   
    
    Eigen::MatrixXd nadirQuat = nadir(g.omega, g.RAAN, g.i, g.tano);

    Eigen::MatrixXd estSunVec = sun_vector_estimate(s.rssd.y, s.rssd.H);
    
    Eigen::MatrixXd b1 = estSunVec;
    Eigen::MatrixXd b2(3,1);

    b2 = s.rmfvA;


    Eigen::MatrixXd b3(3, 1);
    b3 = s.rmfvB;

    Eigen::MatrixXd r1(3, 1);
    r1 << 0.577350269189626,
          0.577350269189626,
         -0.577350269189626;

    Eigen::MatrixXd r2(3, 1);
    
    /*
    reference_magnetic_field_vector refMagVec = igrf(g.reference_year, g.year, g.month, g.day);

    r2(0, 0) = refMagVec.Bx;
    r2(1, 0) = refMagVec.By;
    r2(2, 0) = refMagVec.Bz;

    Eigen::MatrixXd q(4, 1); 
    q = q_method(b1, b2, b3, r1, r2);


    Eigen::Matrix<double, 6, 1> u = Eigen::Matrix<double, 6, 1>::Zero();
    Eigen::Matrix<double, 10, 1> y1 = Eigen::Matrix<double, 10, 1>::Zero();
    
    
    y1[0] = s.rawAngRate(0,0);
    
    y1[1] = s.rawAngRate(1, 0);
    y1[2] = s.rawAngRate(2, 0);
    y1[3] = s.rawAngRate(3, 0);
    y1[4] = s.rawAngRate(4, 0);
    y1[5] = s.rawAngRate(5, 0);
    y1[6] = q(1, 0);
    y1[7] = q(2, 0);
    y1[8] = q(3, 0);
    y1[9] = q(0, 0);

    u[0] = s.rawAngRate(6, 0);
    u[1] = s.rawAngRate(7, 0);
    u[2] = s.rawAngRate(8, 0);
    u[3] = s.rawAngRate(9, 0);
    u[4] = s.rawAngRate(10, 0);
    u[5] = s.rawAngRate(11, 0);
    
    ekf.update(u, y1);
    std::cout << ekf.get_x_hat_kk();
    */
}
