#include "Arduino.h"
#include "LEDController.h"

//Initializes object with PIN number as argument
LEDController::LEDController(int PIN) {
	_PIN = PIN;
	_STATUS = false;
	pinMode(_PIN, OUTPUT);
}

//Returns status of the LED, true if on, false if off
//boolean LEDController:status() {
//	return _STATUS;
//}

//Turns LED on
void LEDController::on() {
	digitalWrite(_PIN, HIGH);
	_STATUS = true;
}

//Turns LED off
void LEDController::off() {
	digitalWrite(_PIN, LOW);
	_STATUS = false;
}

//Toggles LED status
void LEDController::toggle() {
	if (_STATUS) { digitalWrite(_PIN, LOW); } else { digitalWrite(_PIN, HIGH); }
	_STATUS = !_STATUS;
}

//Sets LED intensity via PWM
void LEDController::dim(int INTENSITY) {
	analogWrite(_PIN, INTENSITY);
	_STATUS = true;
}

//Turns LED on, waits for DELAY and then off
void LEDController::onOff(int DELAY) {
	digitalWrite(_PIN, HIGH);
	_STATUS = true;
	delay(DELAY);
	digitalWrite(_PIN, LOW);
	_STATUS = false;
	delay(DELAY);
}

//Gradually changes LED intensity from MIN to MAX, DELAY defines the time for the entire off-on-off cycle
void LEDController::cycleDim(int DELAY, int MIN, int MAX) {
	int _DELAY = DELAY / MAX - MIN; // Defines the duration of each intensity change thru the input DELAY
	analogWrite(_PIN, 0);
	for (int i=MIN; i < MAX; i++) { analogWrite(_PIN, i); delay(_DELAY); }
	for (int i=MAX; i > MIN; i--) { analogWrite(_PIN, i); delay(_DELAY); }
	digitalWrite(_PIN, LOW);
}
