#ifndef LED_RGB_INTERFACE_HPP_
#define LED_RGB_INTERFACE_HPP_

#include "./LED_Interface.hpp"

class LED_RGB_Interface
{
	private:
		LED_Interface* Rled;
		LED_Interface* Gled;
		LED_Interface* Bled;


	public:
		LED_RGB_Interface(LED_Interface* RledInterface,
							LED_Interface* GledInterface,
							LED_Interface* BledInterface) {
			Rled = RledInterface;
			Gled = GledInterface;
			Bled = BledInterface;
		}

		~LED_RGB_Interface() {
			delete Rled;
			delete Gled;
			delete Bled;
		}

		void setRed(bool setOn) {
			if(setOn) {
				Rled->setOn();
			} else {
				Rled->setOff();
			}
		}
		void setGreen(bool setOn) {
			if(setOn) {
				Gled->setOn();
			} else {
				Gled->setOff();
			}
		}
		void setBlue(bool setOn) {
			if(setOn) {
				Bled->setOn();
			} else {
				Bled->setOff();
			}
		}

		void setRGB(bool setRedOn, bool setGreenOn, bool setBlueOn) {
			setRed(setRedOn);
			setGreen(setGreenOn);
			setBlue(setBlueOn);
		}
};

#endif // LED_RGB_INTERFACE_HPP_
