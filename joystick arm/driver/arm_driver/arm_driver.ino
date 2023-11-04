#include <Servo.h>

Servo baseServo;
Servo shoulderServo;
Servo elbowServo;
Servo wristServo;
Servo clawServo;

const int basePin = 11;
const int shoulderPin = 9;
const int elbowPin = 6;
const int wristPin = 5;
const int clawPin = 3;

int theta1 = 90; // base
int theta2 = 90; // shoulder
int theta3 = 0;  // elbow
int theta4 = 0; // wrist
int theta5 = 50; // claw

int range1[2] = {0, 180};  // min : max for theta1 (base)
int range2[2] = {30, 150}; // min : max for theta2 (shoulder)
int range3[2] = {0, 180};  // min : max for theta3 (elbow)
int range4[2] = {0, 180};  // min : max for theta4 (wrist)
int range5[2] = {5, 80};  // min : max for theta5 (claw)

void setup() {
  baseServo.attach(basePin);
  shoulderServo.attach(shoulderPin);
  elbowServo.attach(elbowPin);
  wristServo.attach(wristPin);
  clawServo.attach(clawPin);

  baseServo.write(theta1);
  shoulderServo.write(theta2);
  elbowServo.write(theta3);
  wristServo.write(theta4);
  clawServo.write(theta5);

  Serial.begin(115200);
  Serial.println("Enter values for theta1, theta2, theta3, theta4, and theta5 separated by commas (e.g., 90,45,30,0,10) and press Enter.");
}

void loop() {
  // Check if data is available in the Serial Monitor
  if (Serial.available() > 0) {
    // Read the value entered in the Serial Monitor
    String input = Serial.readStringUntil('\n');
    // Parse the input values using the comma as a delimiter
    int values[5];
    int valueIndex = 0;
    char* token = strtok(const_cast<char*>(input.c_str()), ",");
    
    while (token != NULL && valueIndex < 5) {
      values[valueIndex] = atoi(token);
      token = strtok(NULL, ",");
      valueIndex++;
    }
    
    if (valueIndex == 5) {
      // Assign the parsed values to theta1, theta2, and theta3
      theta1 = values[0];
      theta2 = values[1];
      theta3 = values[2];
      theta4 = values[3];
      theta5 = values[4];

      // Ensure servo angles are within the specified range
      if (theta1 <= range1[0]) {
        theta1 = range1[0];
        baseServo.write(theta1);
      } else if (theta1 >= range1[1]) {
        theta1 = range1[1];
        baseServo.write(theta1);
      } else {
        baseServo.write(theta1);
      }

      if (theta2 <= range2[0]) {
        theta2 = range2[0];
        shoulderServo.write(theta2);
      } else if (theta2 >= range2[1]) {
        theta2 = range2[1];
        shoulderServo.write(theta2);
      } else {
        shoulderServo.write(theta2);
      }

      if (theta3 <= range3[0]) {
        theta3 = range3[0];
        elbowServo.write(theta3);
      } else if (theta3 >= range3[1]) {
        theta3 = range3[1];
        elbowServo.write(theta3);
      } else {
        elbowServo.write(theta3);
      }

      if (theta4 <= range4[0]) {
        theta4 = range4[0];
        wristServo.write(theta4);
      } else if (theta4 >= range4[1]) {
        theta4 = range4[1];
        wristServo.write(theta4);
      } else {
        wristServo.write(theta4);
      }

      if (theta5 <= range5[0]) {
        theta5 = range5[0];
        clawServo.write(theta5);
      } else if (theta5 >= range5[1]) {
        theta5 = range5[1];
        clawServo.write(theta5);
      } else {
        clawServo.write(theta5);
      }
    }
    else {
      Serial.println("Invalid input. Please enter five values separated by commas.");
    }
  }
}
