#ifndef _PID_CONTROLLER
#define _PID_CONTROLLER

#include <mraa.hpp>
#include <cmath>
#include <iostream>
#include <unistd.h>

#include <mutex>
#include <fstream>
#include <cstring>
#include <ctime>
#include <sstream>
#include <chrono>

#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include "../include/FCLogger.hpp"
#include "../include/utility/PidConfig.hpp"
#include "../include/utility/Properties.hpp"

#include "./hardware/interface/Gyro_Interface.hpp"
#include "./hardware/interface/ESC_Controller_Interface.hpp"
#include "./hardware/interface/LED_RGB_Interface.hpp"
#include "./hardware/implementation/upm/Gyro_Impl_MPU6050.hpp"
#include "./hardware/implementation/ESC_Impl_PCA9685.hpp"
#include "./hardware/implementation/LED_Impl.hpp"


#include "FCConstants.hpp"

class PidController
{
public:
	PidController(PidConfig* pidConfig);
	~PidController();
	void setup();
	void calculatePidController();
	void loop(bool rotorsEnabled);
	void iterativeLoop(bool rotorsEnabled);
	//void setPidControllerSetpoint(double* setpointArray);

	void incrementBaselineThrottle(double value);
	void setRollDPS(double dps);
	void setPitchDPS(double dps);
	void setYawDPS(double dps);

	double getNormalizedGyroX();
	double getNormalizedGyroY();
	double getNormalizedGyroZ();
	double getAccelerationX();
	double getAccelerationY();
	double getAccelerationZ();

	void _TEST_ROTORS();
	void _STOP();

	/* FOR USE WITH MULTITHREADING */
	void* p_loop();

	static void*(fcLoopHelper)(void *context) {
		return ((PidController *) context) -> p_loop();
	}

private:
	state currentStatus;

	mraa::I2c* i2c_controller;

	Gyro_Interface* gyro;
	ESC_Controller_Interface* esc_controller;
	LED_RGB_Interface* status_led;

	PidConfig* pidConfigs;

	// RUNNING VARIABLES
};

#endif //_PID_CONTROLLER
