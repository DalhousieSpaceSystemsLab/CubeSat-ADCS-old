#include <Eigen/Dense>

#ifndef DATA_H_
#define DATA_H_


struct gpsData{
	double reference_year;
	uint16_t year;
	uint8_t month;
	uint8_t day;
	double omega;
	double RAAN;
	double i;
	double tano;
	
	double lat_geodetic;
	double phi;
	double H;
};
struct rawSunSensorData{
	Eigen::Matrix<double, 18, 1> y;
	Eigen::Matrix<double, 18, 3> H;
	};
struct sensorData{
        rawSunSensorData rssd;
		Eigen::Matrix<double, 3, 1> rmfvA;
		Eigen::Matrix<double, 3, 1> rmfvB;
		Eigen::Matrix<double, 12, 1> rawAngRate;
};

#endif /*DATA_H_*/




