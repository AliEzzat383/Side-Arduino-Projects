#include <Servo.h>

Servo baseServo;
Servo shoulderServo;
Servo elbowServo;

const int basePin = 3;
const int shoulderPin = 11;
const int elbowPin = 6;

int theta1 = 0; // Initialize theta1

void setup() {
  baseServo.attach(basePin);
  shoulderServo.attach(shoulderPin);
  elbowServo.attach(elbowPin);

  baseServo.write(70);
  shoulderServo.write(0);
  elbowServo.write(0);

  Serial.begin(115200);
  Serial.println("Enter a value for theta1 and press Enter.");
}

void loop() {
  // Check if data is available in the Serial Monitor
  if (Serial.available() > 0) {
    // Read the value entered in the Serial Monitor
    String input = Serial.readStringUntil('\n');
    theta1 = input.toInt(); // Convert the input to an integer

    // Print the updated value of theta1
    Serial.print("Updated theta1: ");
    Serial.println(theta1);
    
    // Set the servo position based on the updated theta1 value
    baseServo.write(theta1);
  }
}
