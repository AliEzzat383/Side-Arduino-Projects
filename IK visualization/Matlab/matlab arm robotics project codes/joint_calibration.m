clc 
clear all
% Arduino setup
arduinoPort = 'COM6';
board = 'Mega2560';
servoLibrary = 'Servo';

% Connect to Arduino
a = arduino(arduinoPort, board, 'libraries', servoLibrary);

% Servo pins on Arduino
shoulderPin = 'D7';
wristPin = 'D11';

% Initialize servo objects
shoulderServo = servo(a, shoulderPin);
wristServo = servo(a, wristPin);

% Set shoulder and wrist angles to 90 degrees
shoulderAngle = 90;
wristAngle = 0;
while(true)
% Write servo angles
writePosition(shoulderServo, shoulderAngle/180);
writePosition(wristServo, wristAngle/180);
end
% Cleanup
clear a;
