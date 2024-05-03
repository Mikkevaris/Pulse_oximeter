#include <Wire.h>
#include "MAX30105.h"

MAX30105 particleSensor;
int val = 0;

void setup() {
	Serial.begin(9600);
	// Initialize sensor
	if (particleSensor.begin() == false) {
		Serial.println("MAX30102 was not found. Please check wiring/power.");
		while (1);
	}

	particleSensor.setup(); //Configure sensor. Use 6.4mA for LED drive
}

void loop() {

  // Test the basic reading
	Serial.print(" R[");
	Serial.print(particleSensor.getRed());
	Serial.print("] IR[");
	Serial.print(particleSensor.getIR());
	Serial.println("]");

  // Determine heart rate

  // Estimate oxygen saturation (SpO2)
  
}