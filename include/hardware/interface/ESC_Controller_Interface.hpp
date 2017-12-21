#ifndef ESC_INTERFACE_HPP_
#define ESC_INTERFACE_HPP_

#include <exception>
#include "stdint.h"
#include "./../../utility/MethodNotImplementedException.hpp"

class ESC_Controller_Interface
{
	protected:
		int rotorCount;

	public:
		ESC_Controller_Interface(int i2c_address) {
			this->rotorCount = 4;
		}

		virtual ~ESC_Controller_Interface() {
		}

		virtual void init() {}

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
