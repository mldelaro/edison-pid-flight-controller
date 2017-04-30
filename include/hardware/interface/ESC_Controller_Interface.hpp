#ifndef ESC_INTERFACE_HPP_
#define ESC_INTERFACE_HPP_

#include <exception>
#include "stdint.h"
#include "./../I2C_Slave_Device.hpp"

class ESC_Controller_Interface : public I2C_Slave_Device
{
	protected:
		int rotorCount;

	public:
		ESC_Controller_Interface(int i2c_address, mraa::I2c* i2c_controller, int rotorCount) : I2C_Slave_Device(i2c_address, i2c_controller) {
			this->rotorCount = rotorCount;
		}

		virtual ~ESC_Controller_Interface() {
		}

		virtual void setPwmFrequency(float frequencyHz) {
			throw methodNotImplementedException;
		}

		virtual void setPwmCycle(int channel, int on, int off) {
			throw methodNotImplementedException;
		}

		virtual void startup() {
			throw methodNotImplementedException;
		}

		virtual void restart() {
			throw methodNotImplementedException;
		}

		virtual void setThrottlePwmDutyCycle(int rotor, int dutyCycle) {
			throw methodNotImplementedException;
		}

		virtual int getThrottlePwmDutyCycleForRotor(int rotor) {
			throw methodNotImplementedException;
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
