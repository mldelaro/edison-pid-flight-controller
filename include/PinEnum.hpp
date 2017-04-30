#ifndef _PIN_ENUM
#define _PIN_ENUM

namespace ArduinoBreakout {
	enum Pin {
		/* Linux pinout translation
		GPIO_0 = 26,
		GPIO_1 = 35,
		GPIO_2 = 13,
		GPIO_3 = 20,
		GPIO_4 = 25,
		GPIO_5 = 14,
		GPIO_7 = 33,
		GPIO_8 = 47,
		GPIO_9 = 21,
		GPIO_10 = 51,
		GPIO_11 = 38,
		GPIO_12 = 50,
		GPIO_13 = 37,
		GPIO_13 = 13,
		*/

		GPIO_1 = 1,
		GPIO_2 = 2,
		GPIO_3 = 3,
		GPIO_4 = 4,
		GPIO_5 = 5,
		GPIO_6 = 6,
		GPIO_7 = 7,
		GPIO_8 = 8,
		GPIO_9 = 9,
		GPIO_10 = 10,
		GPIO_11 = 11,
		GPIO_12 = 12,
		GPIO_13 = 13,

		ANALOG_0 = 0,
		ANALOG_1 = 1,
		ANALOG_2 = 2,
		ANALOG_3 = 3,
		ANALOG_4 = 4,
		ANALOG_5 = 5
	};
}



#endif //_PIN_ENUM
