#ifndef ADCSDATAPACKAGES_HPP
#define ADCSDATAPACKAGES_HPP
/*
This File contains the struct declaration for the ADCS telemetry package as well as OBC package.
This struct contains the data that will be sent back to earth.
*/

//Struct contains Telemetry data to be sent to earth
typedef struct adcsTelemetry {
	//data from sun model
	long sunMdlVX;
	long sunMdlVY;
	long sunMdlVZ;
	//sun vector
	long sunVX;
	long sunVY;
	long sunVZ;
	//data for ananlog magnetometer
	long magAVX;
	long magAVY;
	long magAVZ;
	//data for digital magnetometer
	long magDVX;
	long magDVY;
	long magDVZ;
	//Data for rate gyros
	long rateVX;
	long rateVY;
	long rateVZ;
	//vector
	long roll;
	long pitch;
	long yaw;
	//state of board
	unsigned char systemState;
	unsigned char torquerState;
};

//Struct contains sensor data that will be sent to the OBC
typedef struct adcsToOBC {
	//data for ananlog magnetometer
	long magAVX;
	long magAVY;
	long magAVZ;
	//data for digital magnetometer
	long magDVX;
	long magDVY;
	long magDVZ;
	//18 photodiodes each with 12 bits
    //need better data structure for this
	unsigned photodiode : 12;
	//Data for rate gyros
	long rateVX;
	long rateVY;
	long rateVZ;
	//panel temperature
	unsigned panelTemp : 84;
	//fault state of board
	unsigned short faultState;
};

//Struct contains data sent to adcs board
typedef struct obcToADCS {
	unsigned torqueCycle : 48;
	unsigned char torquerCmd;
	unsigned short stateCmd;
};
#endif