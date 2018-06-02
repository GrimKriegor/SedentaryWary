//Example of LEDController

#include <LEDController.h>


LEDController LED(0);

void setup() {Serial.begin(9600);}

void loop()
{
  Serial.println("w");	
  LED.cycleDim(1000,0,100);
}
