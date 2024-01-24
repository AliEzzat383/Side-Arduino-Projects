#include <Servo.h>

Servo baseServo;
Servo shoulderServo;
Servo elbowServo;
Servo wristRollServo;
Servo wristPitchServo;
Servo clawServo;

const int basePin = 11;
const int shoulderPin = 10;
const int elbowPin = 9;
const int wristRollPin = 6;
const int wristPitchPin = 5;
const int clawPin = 3;

// int theta1 = baseServo.read(); // base
int theta2 = shoulderServo.read(); // shoulder
// int theta3 = elbowServo.read();  // elbow
// int theta4 = wristServoY.read();  // wrist_y
int theta5 = wristPitchServo.read(); // wrist_x
int theta1 = 0; // base
// int theta2 = 0; // shoulder
int theta3 = 0;  // elbow
int theta4 = 0;  // wrist_y
// int theta5 = 0; // wrist_x
int theta6 = 60;  // claw

int range1[2] = {0, 180};  // min : max for theta1 (base)
int range2[2] = {0, 90}; // min : max for theta2 (shoulder)
int range3[2] = {0, 145};  // min : max for theta3 (elbow)
int range4[2] = {0, 180};  // min : max for theta4 (wrist_roll)
int range5[2] = {0, 180};  // min : max for theta5 (wrist_pitch)
int range6[2] = {5, 80};   // min : max for theta6 (claw)

// Define a function to gradually move the servo
void gradualMove(Servo &servo, int currentPos, int targetPos, int stepDelay) {
  int increment = (targetPos > currentPos) ? 1 : -1;

  for (int pos = currentPos; pos != targetPos; pos += increment) {
    servo.write(pos);
    delay(stepDelay);
  }

  // Set the final position
  servo.write(targetPos);
}

void setup() {
  baseServo.attach(basePin);
  shoulderServo.attach(shoulderPin);
  elbowServo.attach(elbowPin);
  wristRollServo.attach(wristRollPin);
  wristPitchServo.attach(wristPitchPin);
  clawServo.attach(clawPin);

  baseServo.write(theta1);
  shoulderServo.write(theta2);
  elbowServo.write(theta3);
  wristRollServo.write(theta4);
  wristPitchServo.write(theta5);
  clawServo.write(theta6);

  Serial.begin(115200);
  Serial.println("Enter values for theta1, theta2, theta3, theta4, theta5, and theta6 separated by commas (e.g., 90,0,30,0,10,20) and press Enter.");
}

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    int values[6];
    int valueIndex = 0;
    char* token = strtok(const_cast<char*>(input.c_str()), ",");
    
    while (token != NULL && valueIndex < 6) {
      values[valueIndex] = atoi(token);
      token = strtok(NULL, ",");
      valueIndex++;
    }
    
    if (valueIndex == 6) {
      theta1 = values[0];
      theta2 = values[1];
      theta3 = values[2];
      theta4 = values[3];
      theta5 = values[4];
      theta6 = values[5];

      if (theta1 <= range1[0]) {
        theta1 = range1[0];
      } else if (theta1 >= range1[1]) {
        theta1 = range1[1];
      } else {
        theta1=theta1;
      }
      if (theta2 <= range2[0]) {
        theta2 = range2[0];
      } else if (theta2 >= range2[1]) {
        theta2 = range2[1];
      } else {
        theta2=theta2;        
      }
      if (theta3 <= range3[0]) {
        theta3 = range3[0];
      } else if (theta3 >= range3[1]) {
        theta3 = range3[1];
      } else {
        theta3=theta3; 
      }
      if (theta4 <= range4[0]) {
        theta4 = range4[0];
      } else if (theta4 >= range4[1]) {
        theta4 = range4[1];
      } else {
        theta4=theta4;
      }
      if (theta5 <= range5[0]) {
        theta5 = range5[0];
      } else if (theta5 >= range5[1]) {
        theta5 = range5[1];
      } else {
        theta5=theta5;
      }
      if (theta6 <= range6[0]) {
        theta6 = range6[0];
      } else if (theta6 >= range6[1]) {
        theta6 = range6[1];
      } else {
        theta6=theta6;
      }
    if ((theta2 == 0 && theta3 > 90)||(theta2 > 120 && theta3 == 0)||(theta6 < 30)){
      Serial.println("Danger Will Robinson");
    }
    else{
      // Gradually move the base servo
        gradualMove(baseServo, baseServo.read(), values[0], 10);

        // Gradually move the shoulder servo
        gradualMove(shoulderServo, shoulderServo.read(), values[1], 10);

        // Gradually move the elbow servo
        gradualMove(elbowServo, elbowServo.read(), values[2], 10);

        // Gradually move the wrist roll servo
        gradualMove(wristRollServo, wristRollServo.read(), theta4, 10);

        // Gradually move the wrist pitch servo
        gradualMove(wristPitchServo, wristPitchServo.read(), theta5, 10);

        // Gradually move the claw servo
        gradualMove(clawServo, clawServo.read(), theta6, 10);
      }
    }
    else {
      Serial.println("Invalid input. Please enter six values separated by commas.");
    }
  }
}
