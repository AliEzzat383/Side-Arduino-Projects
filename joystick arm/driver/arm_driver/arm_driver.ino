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
int theta4 = 90; // wrist
int theta5 = 50; // claw

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

      // Print the updated values of theta1, theta2, and theta3
      Serial.print("Updated theta1: ");
      Serial.println(theta1);
      Serial.print("Updated theta2: ");
      Serial.println(theta2);
      Serial.print("Updated theta3: ");
      Serial.println(theta3);
      Serial.print("Updated theta4: ");
      Serial.println(theta4);
      Serial.print("Updated theta5: ");
      Serial.println(theta5);
      
      // Set the servo positions based on the updated theta values
      baseServo.write(theta1);
      shoulderServo.write(theta2);
      elbowServo.write(theta3);
      wristServo.write(theta4);
      if (theta5 < 20)
      {
        theta5 = 20;
        clawServo.write(theta5);
      }
      else if (theta5 > 40)
      {
        theta5 = 60;
        clawServo.write(theta5);
      }
      }
    else {
      Serial.println("Invalid input. Please enter five values separated by commas.");
    }
  }
}
