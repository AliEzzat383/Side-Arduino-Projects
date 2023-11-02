#include <Servo.h>

// Servo objects
Servo baseServo;
Servo shoulderServo;
Servo elbowServo;

// Servo pin assignments
const int basePin = 12;
const int shoulderPin = 11;
const int elbowPin = 10;

// Link lengths (in centimeters)
const float L1 = 10;
const float L2 = 11.5;
const float L3 = 13.5;

// Target position
float x = 1; // Variable x value
float y = 1
; // Sample y value
float z = 1 ; // Sample z value

void setup() {
  // Attach servo objects to the corresponding pins
  baseServo.attach(basePin);
  shoulderServo.attach(shoulderPin);
  elbowServo.attach(elbowPin);
//  baseServo.write(0);
//  shoulderServo.write(0);
//  elbowServo.write(0);
  // Start serial communication
  Serial.begin(9600);
}

void loop() {
  // Check if x and y values are within the valid range
  if (x > -(L2 + L3) && x < (L2 + L3) && y > -(L2 + L3) && y < (L2 + L3)) {
    // Check if z value is within the valid range
    if (z >= L1 && z <= (L1 + L2 + L3)) {
      // Calculate the joint angles using inverse kinematics
      float r = sqrt(x * x + y * y);
      float s = z - L1;
      float w = sqrt(s * s + r * r);
      float D = (w * w - L2 * L2 - L3 * L3) / (2 * L2 * L3);
      float theta3 = atan2(sqrt(1 - D * D), D) * (180 / PI);
      float theta1 = atan2(y, x) * (180 / PI);
      float gamma = atan2(z - L1, r) * (180 / PI);
      float alpha = atan2(L3 * sin(theta3 * (PI / 180)), L2 + L3 * cos(theta3 * (PI / 180))) * (180 / PI);
      float theta2 = gamma - alpha;

      // Set servo positions
//      float ang = 45;
//      theta1 =ang;
//      theta2 =ang;
//      theta3 =ang;
      baseServo.write(-90);
      shoulderServo.write(0);
      elbowServo.write(90);

      // Print joint angles
      Serial.print("theta1: ");
      Serial.print(theta1);
      Serial.print(", theta2: ");
      Serial.print(theta2);
      Serial.print(", theta3: ");
      Serial.println(theta3);

      // Wait for the servos to reach the target positions
      delay(1000);
    } else {
      Serial.println("Error: Invalid z value");
    }
  } else {
    Serial.println("Error: Invalid x or y value");
  }

  // Delay before repeating the loop
  delay(1000);
}
