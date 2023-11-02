#include <Servo.h>

Servo baseServo;
Servo shoulderServo;
Servo elbowServo;

const int basePin = 11;
const int shoulderPin = 9;
const int elbowPin = 6;

int theta1 = 0;
int theta2 = 0;
int theta3 = 0;

void setup() {
  baseServo.attach(basePin);
  shoulderServo.attach(shoulderPin);
  elbowServo.attach(elbowPin);

  baseServo.write(0);
  shoulderServo.write(0);
  elbowServo.write(0);

  Serial.begin(115200);
  Serial.println("Enter values for theta1, theta2, and theta3 separated by commas (e.g., 90,45,30) and press Enter.");
}

void loop() {
  // Check if data is available in the Serial Monitor
  if (Serial.available() > 0) {
    // Read the value entered in the Serial Monitor
    String input = Serial.readStringUntil('\n');
    // Parse the input values using the comma as a delimiter
    int values[3];
    int valueIndex = 0;
    char* token = strtok(const_cast<char*>(input.c_str()), ",");
    while (token != NULL && valueIndex < 3) {
      values[valueIndex] = atoi(token);
      token = strtok(NULL, ",");
      valueIndex++;
    }
    
    if (valueIndex == 3) {
      // Assign the parsed values to theta1, theta2, and theta3
      theta1 = values[0];
      theta2 = values[1];
      theta3 = values[2];

      // Print the updated values of theta1, theta2, and theta3
      Serial.print("Updated theta1: ");
      Serial.println(theta1);
      Serial.print("Updated theta2: ");
      Serial.println(theta2);
      Serial.print("Updated theta3: ");
      Serial.println(theta3);

      // Set the servo positions based on the updated theta values
      baseServo.write(theta1);
      shoulderServo.write(theta2);
      elbowServo.write(theta3);
    }
    else {
      Serial.println("Invalid input. Please enter three values separated by commas.");
    }
  }
}
