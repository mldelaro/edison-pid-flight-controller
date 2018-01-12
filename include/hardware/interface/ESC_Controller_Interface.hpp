#ifndef ESC_INTERFACE_HPP_
#define ESC_INTERFACE_HPP_

#include <exception>
#include "stdint.h"
#include "./../../utility/MethodNotImplementedException.hpp"
#include "./../I2C_Slave_Device.hpp"

class ESC_Controller_Interface : protected I2C_Slave_Device
{
	protected:
		int rotorCount;

	public:
		ESC_Controller_Interface(int i2c_address) {
			this->rotorCount = 4;
		}

		ESC_Controller_Interface(int i2c_address, mraa::I2c* i2c_controller, int rotorCount) {
			this->rotorCount = 4;
			this->i2c = i2c_controller;
			this->i2c_address = i2c_address;
		}

		virtual ~ESC_Controller_Interface() {
		}

		virtual void init() {}

		virtual void setPwmFrequency(float frequencyHz) {
			throw new METHOD_NOT_IMPLEMENTED();
		}

		virtual void setPwmCycle(int channel, int on, int off) {
			throw new METHOD_NOT_IMPLEMENTED();
		}

		virtual void startup() {
			throw new METHOD_NOT_IMPLEMENTED();
		}

		virtual void restart() {
			throw new METHOD_NOT_IMPLEMENTED();
		}

		virtual void setThrottlePwmDutyCycle(int rotor, int dutyCycle) {
			throw new METHOD_NOT_IMPLEMENTED();
		}

		virtual int getThrottlePwmDutyCycleForRotor(int rotor) {
			throw new METHOD_NOT_IMPLEMENTED();
			return -1;
		}

		enum QuadRotor {
			FrontLeft = 0,
			FrontRight = 1,
			BackLeft = 2,
			BackRight = 3
		};
};

#endif // ESC_INTERFACE_HPP_
