//
// LEDController library by Grim Kriegor
// grimkriegor@opmbx.org
// Version 150125
//
// For easier handling of LEDs
//

#include "Arduino.h"

#ifndef LEDController_h
#define LEDController_h



class LEDController
{
	private:
		int _PIN;
		boolean _STATUS;
	public:
		LEDController(int PIN);
		boolean status();
		void on();
		void off();
		void toggle();
		void dim(int INTENSITY=50);
		void onOff(int DELAY=100);
		void cycleDim(int DELAY=10, int MIN=0, int MAX=255);
};

#endif


	
