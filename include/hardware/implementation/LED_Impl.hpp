#ifndef LED_IMPL_H_
#define LED_IMPL_H_

#include <mraa.hpp>

#include "./../interface/LED_Interface.hpp"

class LED_Impl : public LED_Interface {
private:
	mraa::Gpio* gpioPin;
	bool isOn;
	bool hasMraaPinReference; // flags if pin was successfully allocated

public:
	LED_Impl(int pin) : LED_Interface(pin) {
		isOn = false;
		hasMraaPinReference = true;
		gpioPin = new mraa::Gpio(pin, true, false);
		if(gpioPin == NULL) {
			hasMraaPinReference = false;
		}
		if (gpioPin->dir(mraa::DIR_OUT) != mraa::SUCCESS) {
			hasMraaPinReference = false;
		}
	}

	~LED_Impl() {
	}

	bool getStatus() {
		return isOn;
	}

	void setOn() {
		isOn = true;
		if(hasMraaPinReference) {
			gpioPin->write(1);
		}
	}
	void setOff() {
		isOn = false;
		if(hasMraaPinReference) {
			gpioPin->write(0);
		}
	}
	void toggle() {
		isOn = true;
		if(hasMraaPinReference) {
			if(isOn) {
				gpioPin->write(0);
			} else {
				gpioPin->write(1);
			}
		}
	}
};

#endif // LED_IMPL_H_
