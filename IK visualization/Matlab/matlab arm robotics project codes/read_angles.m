clc;
clear;

% DH Parameters [theta d a alpha]
% DH_parameters = [0 0 1.025 0;
%                  0 pi/2 0.45 0;
%                  1.65 0 0 0;
%                  1.65 0 0 0];

% Arduino setup
arduinoPort = 'COM6';
board = 'Mega2560';
servoLibrary = 'Servo';

% Connect to Arduino
a = arduino(arduinoPort, board, 'libraries', servoLibrary);

% Servo pins on Arduino
basePin = 'D5';
shoulderPin = 'D7';
elbowPin = 'D12';

% Initialize servo objects
baseServo = servo(a, basePin);
shoulderServo = servo(a, shoulderPin);
elbowServo = servo(a, elbowPin);

while(true)
% Specify servo angles (in degrees)
baseAngle = 0;       % Set base servo angle to 90 degrees
shoulderAngle = 90;   % Set shoulder servo angle to 45 degrees
elbowAngle = 30;       % Set elbow servo angle to 0 degrees
% 
% Write servo angles
writePosition(baseServo, baseAngle/180);         % Convert angle to servo position
writePosition(shoulderServo, shoulderAngle/180); % Convert angle to servo position
writePosition(elbowServo, elbowAngle/180);       % Convert angle to servo position

% Wait for servos to reach the desired positions
pause(2);

   % Read servo angles
baseAngleRead = readPosition(baseServo) * 180;
shoulderAngleRead = readPosition(shoulderServo) * 180;
elbowAngleRead = readPosition(elbowServo) * 180; 

% Display arm configuration
    fprintf('Base angle: (read: %.2f degrees)\n', baseAngleRead);
    fprintf('Shoulder angle: (read: %.2f degrees)\n', shoulderAngleRead);
    fprintf('Elbow angle:(read: %.2f degrees)\n', elbowAngleRead);
end
% Display arm configuration

% Cleanup
clear a;
