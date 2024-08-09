#include <AccelStepper.h>

// Define click.
#define CLICK_PIN 21
#define CLICK_DURATION_MS 50

// Stepper definitions
#define DIR_PIN 19   // Direction
#define STEP_PIN 18  // Step
#define EN_PIN 4     // Driver enable
#define MS1_PIN 16   // MS1
#define MS2_PIN 17   // MS2


// Stepper definition
AccelStepper stepper = AccelStepper(stepper.DRIVER, STEP_PIN, DIR_PIN);

String myCmd;

void setup() {
	// Setup serial
	Serial.begin(115200);

	// Setup stepper
	pinMode(DIR_PIN, OUTPUT);
	pinMode(STEP_PIN, OUTPUT);
	pinMode(EN_PIN, OUTPUT);   // Driver enable
	pinMode(MS1_PIN, OUTPUT);  // MS1
	pinMode(MS2_PIN, OUTPUT);  // MS2
	digitalWrite(EN_PIN, LOW);
	digitalWrite(MS1_PIN, LOW);
	digitalWrite(MS2_PIN, LOW);

	stepper.setMaxSpeed(102.00);
	stepper.setAcceleration(101.00);
}

void loop() {
	myCmd = "";
	if (Serial.available() > 0) {
		myCmd = Serial.readStringUntil('\r');
		Serial.print("Got new position: ");
		Serial.print(myCmd);
		Serial.println(".");
		stepper.runToNewPosition(myCmd.toInt());
		Serial.println("Done moving, going home.");
		stepper.runToNewPosition(0);
	}
}
// Setup stepper
