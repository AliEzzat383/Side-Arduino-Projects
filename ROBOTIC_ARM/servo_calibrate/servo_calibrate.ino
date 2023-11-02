#include <Servo.h>

Servo elbow;  // Create a Servo object named "elbow"

const int elbowPin = 9;  // Define the servo pin

void setup() {
  elbow.attach(elbowPin);  // Attach the servo to the specified pin
}

void loop() {
  int angle = 0;  // Set the desired angle (in degrees)

  // Move the "elbow" servo to angle zero
  elbow.write(angle);
  
  delay(1000);  // Delay for 1 second
}
