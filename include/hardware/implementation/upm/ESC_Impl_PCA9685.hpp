#include <pca9685.hpp>
#ifndef ESC_IMPL_9685_H_
#define ESC_IMPL_9685_H_

#include <mraa.hpp>

#include "./../../interface/ESC_Controller_Interface.hpp"

const int PCA9685_TX_RX_BUFFER_LENGTH = 50;
const int PCA9685_I2C_ADDRESS = 0x40;
const int PCA9685_RESOLUTION = 4096;
const int PCA9685_CLOCK_HZ = 25000000;

const int REG_MODE_1 = 0x00;
const int REG_MODE_2 = 0x01;
const int REG_ALL_CALL_ADR = 0x05;
const int REG_LED_0_ON_L = 0x06;
const int REG_LED_0_ON_H = 0x07;
const int REG_LED_ALL_ON_L = 0x0FA;
const int REG_LED_ALL_OFF_L = 0xFC;
const int REG_PRESCALE = 0xFE;

const int ENABLE_ALL_CALL = 0x01;
const int ENABLE_OUTDRV = 0x04;
const int ENABLE_INVRT = 0x08;
const int ENABLE_RESET = 0x80;
const int ENABLE_SLEEP = 0x10;

const int ESC_PWM_FREQ_HZ = 50;

#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))

class ESC_Impl_PCA9685 : public ESC_Controller_Interface {
private:
	upm::PCA9685* pca9685;
	int* rotorThrottle;

	void _RESTART() {
		if(pca9685->setModeSleep(true)) {
			std::cout << "PCA9685 - Put in sleep mode..." << std::endl;
		} else {
			std::cout << "PCA9685 - FAILED to put in sleep mode..." << std::endl;
		}

		setPwmFrequency(ESC_PWM_FREQ_HZ);

		if(pca9685->setModeSleep(false)) {
			std::cout << "PCA9685 - waking up..." << std::endl;
		} else {
			std::cout << "PCA9685 - FAILED to wake up..." << std::endl;
		}
	}

	void SET_DRIVER_ALL_OFF() {
	}

	void SET_DRIVER_ALL_ON() {
	}

	void setPwmCycle(int channel, int on, int off) {
		if(!pca9685->ledOnTime(channel, on)) {
			std::cout << "PCA9685 - FAILED to set led on-time..." << std::endl;
		}
		if(!pca9685->ledOffTime(channel, off)) {
			std::cout << "PCA9685 - FAILED to set led off-time..." << std::endl;
		}
	}

public:

	ESC_Impl_PCA9685(int i2c_address) : ESC_Controller_Interface(i2c_address) {
		std::cout << "PCA9685 - Constructor via UPM" << std::endl;
		rotorThrottle = new int[4]; // rotor throttle for each rotor on quadcopter
		pca9685 = new upm::PCA9685(PCA9685_I2C_BUS, PCA9685_I2C_ADDRESS);
	}

	void startup() {
	}

	void setPwmFrequency(float frequencyHz) {
		//pca9685->setModeSleep(true);
		if(pca9685->setPrescaleFromHz(frequencyHz)) {
			std::cout << "PCA9685 - Reset pwm frequency..." << std::endl;
		} else {
			std::cout << "PCA9685 - Failed to Reset pwm frequency..." << std::endl;
		}
		//pca9685->setModeSleep(false);
	}

	void setThrottlePwmDutyCycle(int rotor, int dutyCycle) {
		rotorThrottle[rotor] = dutyCycle;
		setPwmCycle(rotor, 0, dutyCycle);
	}

	int getThrottlePwmDutyCycleForRotor(int rotor, int dutyCycle) {
		if(rotor >= rotorCount) {
			return -1;
		}
		return rotorThrottle[rotor];
	}

	void init() {
		_RESTART();
	}

	~ESC_Impl_PCA9685() {
		std::cout << "PCA9685 - DESTROY" << std::endl;
		setPwmCycle(0, 0, 1000);
		setPwmCycle(1, 0, 1000);
		setPwmCycle(2, 0, 1000);
		setPwmCycle(3, 0, 1000);
	}

};

#endif // ESC_IMPL_9685_H_
