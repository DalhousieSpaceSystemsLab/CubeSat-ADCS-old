#include <Eigen/Dense>

#ifndef DATA_H_
#define DATA_H_


struct gpsData{
	double reference_year;
	double year;
	double month;
	double day;
	double omega;
	double RAAN;
	double i;
	double tano;
};
struct rawSunSensorData{
		Eigen::Matrix<double, 18, 3> H;
		Eigen::Matrix<double, 18, 1> y;
	};
struct sensorData{
        rawSunSensorData rssd;
		Eigen::Matrix<double, 3, 1> rmfvA;
		Eigen::Matrix<double, 3, 1> rmfvB;
		Eigen::Matrix<double, 12, 1> rawAngRate;
};

#endif /*DATA_H_*/




