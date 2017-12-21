#ifndef GYRO_INTERFACE_HPP_
#define GYRO_INTERFACE_HPP_

#include <exception>
#include "stdint.h"
#include "./../../utility/MethodNotImplementedException.hpp"

class Gyro_Interface //: public I2C_Slave_Device
{
	public:

		Gyro_Interface(int i2c_address) {
		}

		Gyro_Interface(int i2c_address, mraa::I2c* i2c_controller) {
		}

		virtual void init() {}

		virtual ~Gyro_Interface() {
		}

		virtual void callibrateGyro(int sampleCount) {
			throw new METHOD_NOT_IMPLEMENTED();
		}

		virtual void read() {
			throw new METHOD_NOT_IMPLEMENTED();
		}

		virtual double getRoll_DPS () {
			throw new METHOD_NOT_IMPLEMENTED();
			return -1;
		}

		virtual double getPitch_DPS () {
			throw new METHOD_NOT_IMPLEMENTED();
			return -1;
		}

		virtual double getYaw_DPS () {
			throw new METHOD_NOT_IMPLEMENTED();
			return -1;
		}

		virtual double getAccX_G () {
			throw new METHOD_NOT_IMPLEMENTED();
			return -1;
		}

		virtual double getAccY_G () {
			throw new METHOD_NOT_IMPLEMENTED();
			return -1;
		}

		virtual double getAccZ_G () {
			throw new METHOD_NOT_IMPLEMENTED();
			return -1;
		}
};

#endif // GYRO_INTERFACE_HPP_
