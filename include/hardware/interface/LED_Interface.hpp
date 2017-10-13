#ifndef LED_INTERFACE_HPP_
#define LED_INTERFACE_HPP_

class LED_Interface
{
	public:
		LED_Interface(int pin) {
		}

		virtual ~LED_Interface() {
		}

		virtual bool getStatus() {
			return 0;
		}

		virtual void setOn() {
		}
		virtual void setOff() {
		}
		virtual void toggle() {
		}
};

#endif // LED_INTERFACE_HPP_
