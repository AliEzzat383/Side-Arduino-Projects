#include <Servo.h>

// Servo objects
Servo baseServo;
Servo shoulderServo;
Servo elbowServo;

// Servo pin assignments
const int basePin = 12;     // Adjust the pin numbers as per your setup
const int shoulderPin = 11; // Adjust the pin numbers as per your setup
const int elbowPin = 10;    // Adjust the pin numbers as per your setup
// angles
double theta1 = 0;
double theta2 = 0;
double theta3 = 0;
void setup()
{
  // Attach servo objects to the corresponding pins
  baseServo.attach(basePin);
  shoulderServo.attach(shoulderPin);
  elbowServo.attach(elbowPin);

  // Set initial positions for the servos
  // Note: These positions will be overwritten in the loop()
}

void loop()
{
  // Set the homing position
  //Homing
  /*baseServo.write(0);    
  shoulderServo.write(90); 
  elbowServo.write(0);
  delay(1000);
  //+ve X
  baseServo.write(90);    
  shoulderServo.write(0); 
  elbowServo.write(90);
  delay(1000);
  //-ve X
  baseServo.write(90);    
  shoulderServo.write(180); 
  elbowServo.write(90);
  delay(1000);*/
  // Max Z
  baseServo.write(90);    
  shoulderServo.write(90); 
  elbowServo.write(90);
  delay(1000);
  // Add your code here if needed

  // Delay before repeating the loop
 
}
