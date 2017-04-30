#ifndef GYRO_INTERFACE_HPP_
#define GYRO_INTERFACE_HPP_

#include <exception>
#include "stdint.h"
#include "./../I2C_Slave_Device.hpp"

class Gyro_Interface : public I2C_Slave_Device
{
	public:
		Gyro_Interface(int i2c_address, mraa::I2c* i2c_controller) : I2C_Slave_Device(i2c_address, i2c_controller) {
		}

		virtual void read() {
			throw methodNotImplementedException;
		}
		virtual ~Gyro_Interface() {
		}

		virtual int16_t getAcc_x() {
			throw methodNotImplementedException;
			return -1;
		}
		virtual int16_t getAcc_y() {
			throw methodNotImplementedException;
			return -1;
		}
		virtual int16_t getAcc_z() {
			throw methodNotImplementedException;
			return -1;
		}

		virtual int16_t getGyro_roll() {
			throw methodNotImplementedException;
			return -1;
		}
		virtual int16_t getGyro_pitch() {
			throw methodNotImplementedException;
			return -1;
		}
		virtual int16_t getGyro_yaw() {
			throw methodNotImplementedException;
			return -1;
		}

		virtual int16_t getTemp() {
			throw methodNotImplementedException;
			return -1;
		}


		virtual float getOutputForOneDegreePerSecond() {
			throw methodNotImplementedException;
			return -1;
		}
};

#endif // GYRO_INTERFACE_HPP_
