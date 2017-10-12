#include "../include/FCConstants.hpp"

/* CALCULATION CONSTANTS */
const float fc_constants::RATIO_RADIAN_TO_DEGREE = 57.296;
const float fc_constants::RATIO_DEGREE_TO_RADIAN = 0.000001066; //3.142 / 180

/* HARDWARE CONFIGURATIONS*/
//const int PIN_LED_WARNING = ArduinoBreakout::GPIO_13;
const int fc_constants::PIN_BATTERY_VOLTAGE = ArduinoBreakout::ANALOG_0;
const int fc_constants::I2C_DEVICE_COUNT = 2;
const int fc_constants::I2C_ADDR_GYRO = 0x68;
const int fc_constants::I2C_ADDR_ESC_CONTROLLER = 0x40;


/* FLIGHT CONTROLLER CONFIGURATIONS */
double fc_constants::PID_PROPORTIONAL_GAIN[3] = {0, 0, 3};	// Proportional-Gain for roll, pitch, yaw
double fc_constants::PID_INTEGRAL_GAIN[3] = {0, 0, 0.02};		// Integral-Gain for roll, pitch, yaw
double fc_constants::PID_DERIVATIVE_GAIN[3] = {0, 0, 0};	// Derivative-Gain for roll, pitch, yaw
double fc_constants::PID_MAX_OUTPUT[3] = {0, 0, 0};					// limit PID output
const int fc_constants::GYRO_SAMPLE_COUNT = 2000;							// number of samples to take for callibration
const float fc_constants::GYRO_1DPS_RAW_OUTPUT = 65.5; 					// from Gyro datasheet (degrees/sec value)
const int fc_constants::MAX_RECEIVER_THROTTLE = 2000;
const int fc_constants::CORRECTION_FREQUENCY_HZ = 250;
