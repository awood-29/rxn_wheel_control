// Arduino DC Motor Test Script
// Author: Andrew Wood
// adapted from https://lastminuteengineers.com/l298n-dc-stepper-driver-arduino-tutorial/

// Motor A connections
int enA = 9; //enable A to PWM
int in1 = 8; // digital non PWM
int in2 = 7; // pwm? not yet

// Motor B connections
/*int enB = 3;
int in3 = 5;
int in4 = 4;
*/

void setup() {
	Serial.begin(9600);
  // Set all the motor control pins to outputs
	pinMode(enA, OUTPUT);
	pinMode(in1, OUTPUT);
	pinMode(in2, OUTPUT);
	
	// Turn off motors - Initial state
	digitalWrite(in1, LOW);
	digitalWrite(in2, LOW);
  Serial.print("Hello World\n");
}

void loop() {
	directionControl();
	delay(1000);
	speedControl();
	delay(1000);
  Serial.print("done");
}

// This function lets you control spinning direction of motors
void directionControl() {
	// Set motors to maximum speed
	// For PWM maximum possible values are 0 to 255
	analogWrite(enA, 255);

	// Turn on motor A & B
	digitalWrite(in1, HIGH);
	digitalWrite(in2, LOW);
	delay(2000);
	
	// Now change motor directions
	digitalWrite(in1, LOW);
	digitalWrite(in2, HIGH);
	delay(2000);
	
	// Turn off motors
	digitalWrite(in1, LOW);
	digitalWrite(in2, LOW);
}

// This function lets you control speed of the motors
void speedControl() {
	// Turn on motors
	digitalWrite(in1, LOW);
	digitalWrite(in2, HIGH);
	
	// Accelerate from zero to maximum speed
	for (int i = 0; i < 256; i++) {
		analogWrite(enA, i);
		delay(20);
	}
	
	// Decelerate from maximum speed to zero
	for (int i = 255; i >= 0; --i) {
		analogWrite(enA, i);
		delay(20);
	}
	
	// Now turn off motors
	digitalWrite(in1, LOW);
	digitalWrite(in2, LOW);
}
