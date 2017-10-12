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

#include "./hardware/I2C_Slave_Device.hpp"
#include "./hardware/interface/Gyro_Interface.hpp"
#include "./hardware/interface/ESC_Controller_Interface.hpp"
#include "./hardware/implementation/Gyro_Impl_MPU6050.hpp"
#include "./hardware/implementation/ESC_Impl_PCA9685.hpp"

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

	void incrementBaselineThrottle(int value);
	void setRollDPS(int dps);
	void setPitchDPS(int dps);
	void setYawDPS(int dps);

	void _TEST_ROTORS();
	void _STOP();

	/* FOR USE WITH MULTITHREADING */
	void* p_loop();

	static void*(fcLoopHelper)(void *context) {
		return ((PidController *) context) -> p_loop();
	}

private:
	state currentStatus;

	I2C_Slave_Device** i2c_slaveDevices;
	mraa::I2c* i2c_controller;

	Gyro_Interface* gyro;
	ESC_Controller_Interface* esc_controller;

	PidConfig* pidConfigs;

	// RUNNING VARIABLES
};

#endif //_PID_CONTROLLER
