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

		virtual ~Gyro_Interface() {
		}

		virtual void callibrateGyro(int sampleCount) {
			throw methodNotImplementedException;
		}

		virtual void read() {
			throw methodNotImplementedException;
		}

		virtual double getRoll_DPS () {
			throw methodNotImplementedException;
			return -1;
		}

		virtual double getPitch_DPS () {
			throw methodNotImplementedException;
			return -1;
		}

		virtual double getYaw_DPS () {
			throw methodNotImplementedException;
			return -1;
		}

		virtual double getAccX_G () {
			throw methodNotImplementedException;
			return -1;
		}

		virtual double getAccY_G () {
			throw methodNotImplementedException;
			return -1;
		}

		virtual double getAccZ_G () {
			throw methodNotImplementedException;
			return -1;
		}
};

#endif // GYRO_INTERFACE_HPP_
