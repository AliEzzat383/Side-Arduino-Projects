#include <Servo.h>

Servo baseServo;
Servo shoulderServo;
Servo elbowServo;

const int ReadbasePin = A0;      // Analog input pin for the base potentiometer
const int basePin = 12;
const int shoulderPin = 11;
const int elbowPin = 10;
const int ReadelbowPin = A0;     // Analog input pin for the elbow potentiometer

double l1 = 40.5;
double l2 = 12.5;
double l3 = 14.5;

const double RADIANS_TO_DEGREES = 57.3;  // Approximate conversion of radians to degrees

void setup() {
  Serial.begin(115200);
  baseServo.attach(basePin);
  shoulderServo.attach(shoulderPin);
  elbowServo.attach(elbowPin);

  delay(2000);  // Wait for 2 seconds
}

void loop() {
  // Read the analog input from A0 (base potentiometer)
  int basePotValue = analogRead(ReadbasePin);

  // Read the analog input from A4 (elbow potentiometer)
  int elbowPotValue = analogRead(ReadelbowPin);

  // Map the analog input values to the servo angle range (0 to 180 degrees)
  int basePosition = map(basePotValue, 0, 1023, 0, 180);
  int elbowPosition = map(elbowPotValue, 0, 1023, 0, 180);

  // Move the base and elbow servos to the calculated positions
  //baseServo.write(basePosition);
  elbowServo.write(elbowPosition);

  // Print potentiometer values for debugging
  Serial.print("Base Pot Value: ");
  Serial.print(basePotValue);
  Serial.print(", Elbow Pot Value: ");
  Serial.print(elbowPotValue);
  Serial.print(", Base Position: ");
  Serial.print(basePosition);
  Serial.print(", Elbow Position: ");
  Serial.println(elbowPosition);

  // Rest of the code for the square drawing using inverse kinematics remains unchanged...
}
