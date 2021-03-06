#ifndef ESC_IMPL_9685_H_
#define ESC_IMPL_9685_H_

#include <mraa.hpp>

#include "./../interface/ESC_Controller_Interface.hpp"
#include "./../I2C_Slave_Device.hpp"

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

const int ESC_PWM_FREQ_HZ = 300;

#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))

class ESC_Impl_PCA9685 : public ESC_Controller_Interface {
private:
	int* rotorThrottle;

	void _RESTART() {
		std::cout << "PCA9685 - Restarting..." << std::endl;

		rx_tx_buf[0] = REG_MODE_1; // restart
		rx_tx_buf[1] = 0;
		writeBuffer(rx_tx_buf, 2);
	}

	void SET_DRIVER_ALL_OFF() {
		rx_tx_buf[0] = REG_LED_ALL_ON_L;	// Write 0 to ALL_LED_ON Register
		rx_tx_buf[1] = 0x00;				// LSB set to 0
		rx_tx_buf[2] = 0x00;				// MSB set to 0
		rx_tx_buf[3] = 0x00;				// Write 0 to ALL_LED_OFF - LSB set to 0
		rx_tx_buf[4] = 0x00;				// MSB set to 0
		writeBuffer(rx_tx_buf, 5);
	}

	void SET_DRIVER_ALL_ON() {
		rx_tx_buf[0] = REG_LED_ALL_ON_L;	// Write 0 to ALL_LED_ON Register
		rx_tx_buf[1] = 0x00;				// LSB set to 0
		rx_tx_buf[2] = 0x00;				// MSB set to 0
		rx_tx_buf[3] = 0xFF;				// Write 0 to ALL_LED_OFF - LSB set to 0
		rx_tx_buf[4] = 0xFF;				// MSB set to 0
		writeBuffer(rx_tx_buf, 5);
	}

	void setPwmCycle(int channel, int on, int off) {
		uint8_t destRegister = REG_LED_0_ON_L + 4*channel;

		// limit input value to 12-bit resolution
		if(on > 4095) { on = 4095; }
		if(on < 0) { on = 0; }

		if(off > 4095) { off = 4095; }
		if(off < 0) { off = 0; }

		rx_tx_buf[0] = destRegister;			// Write 0 to LED_[channel] Register
		rx_tx_buf[1] = on & 0xFF;				// LSB set to 0
		rx_tx_buf[2] = (on & 0xFF00) >> 8;		// MSB set to 0
		rx_tx_buf[3] = off & 0xFF;				// LSB set to 0
		rx_tx_buf[4] = (off & 0xFF00) >> 8;		// MSB set to 0
		writeBuffer(rx_tx_buf, 5);
	}

public:

	ESC_Impl_PCA9685(int i2c_address, mraa::I2c* i2c_controller, int rotorCount) : ESC_Controller_Interface(i2c_address, i2c_controller, rotorCount) {
		std::cout << "PCA9685 - Constructor" << std::endl;
		rotorThrottle = new int[rotorCount];
	}

	void startup() {
	}

	void setPwmFrequency(float frequencyHz) {
		std::cout << "PCA9685 - Setting PWM Frequency to ~" << (int)frequencyHz << std::endl;
		frequencyHz *= 0.9;
		float prescaleValue = PCA9685_CLOCK_HZ;
		prescaleValue /= PCA9685_RESOLUTION;
		prescaleValue /= frequencyHz;
		prescaleValue -= 1;

		bool didSucceed = false;

		while(!didSucceed) {
			uint8_t prescale = floor(prescaleValue);
			uint8_t original_mode1 = readByteValueFromAddress(REG_MODE_1);
			uint8_t new_mode1 = (original_mode1 & 0x7F) | ENABLE_SLEEP; // set sleep bit

			rx_tx_buf[0] = REG_MODE_1; // sleep
			rx_tx_buf[1] = new_mode1;
			writeBuffer(rx_tx_buf, 2);

			rx_tx_buf[0] = REG_PRESCALE; // set prescaler
			rx_tx_buf[1] = prescale;
			writeBuffer(rx_tx_buf, 2);

			if(readByteValueFromAddress(REG_PRESCALE) != prescale) {
				std::cout << "PCA9685 - Failed to set Prescale Register" << std::endl;
			} else {
				didSucceed = true;
			}

			rx_tx_buf[0] = REG_MODE_1; // restore mode1
			rx_tx_buf[1] = original_mode1;
			writeBuffer(rx_tx_buf, 2);

			rx_tx_buf[0] = REG_MODE_1; // restore mode1
			rx_tx_buf[1] = original_mode1 | 0xA1; //enable auto increment
			writeBuffer(rx_tx_buf, 2);
		}
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
