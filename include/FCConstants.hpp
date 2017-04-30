#ifndef _FLIGHT_CONTROLLER_CONSTANTS
#define _FLIGHT_CONTROLLER_CONSTANTS

#define PIN_LED_WARNING 13
#define ANALOG_PIN_BATTERY_VOLTAGE 0

#include "./PinEnum.hpp"

namespace fc_constants {

	/* CALCULATION CONSTANTS */
	extern const float RATIO_RADIAN_TO_DEGREE;
	extern const float RATIO_DEGREE_TO_RADIAN; //3.142 / 180

	/* HARDWARE CONFIGURATIONS*/
	extern const int PIN_BATTERY_VOLTAGE;
	extern const int I2C_DEVICE_COUNT;
	extern const int I2C_ADDR_GYRO;
	extern const int I2C_ADDR_ESC_CONTROLLER;


	/* FLIGHT CONTROLLER CONFIGURATIONS */
	extern double PID_PROPORTIONAL_GAIN[3];	// Proportional-Gain for roll, pitch, yaw
	extern double PID_INTEGRAL_GAIN[3];		// Integral-Gain for roll, pitch, yaw
	extern double PID_DERIVATIVE_GAIN[3];	// Derivative-Gain for roll, pitch, yaw
	extern double PID_MAX_OUTPUT[3];					// limit PID output
	extern const int GYRO_SAMPLE_COUNT;							// number of samples to take for callibration
	extern const float GYRO_1DPS_RAW_OUTPUT; 					// from Gyro datasheet (degrees/sec value)
	extern const int MAX_RECEIVER_THROTTLE;
	extern const int CORRECTION_FREQUENCY_HZ;
}

enum state {
	INIT,
	SETUP,
	GYRO_CALLIBRATION,
	ESC_INIT,
	START_MOTORS,
	RUNNING
};

enum eulerAngles {
	ROLL = 0,
	PITCH = 1,
	YAW = 2
};

enum axes {
	X = 0,
	Y = 1,
	Z = 2
};

#endif // _FLIGHT_CONTROLLER_CONSTANTS
