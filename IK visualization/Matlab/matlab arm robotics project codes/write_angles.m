clc; clear all;

% Create an Arduino object
a = arduino('COM6', 'Mega2560');

% Define servo pin numbers
baseServoPin = 'D3';
shoulderServoPin = 'D7';
elbowServoPin = 'D12';

% Set up servo objects
baseServo = servo(a, baseServoPin);
shoulderServo = servo(a, shoulderServoPin);
elbowServo = servo(a, elbowServoPin);

% Read and print initial servo positions
basePos = readPosition(baseServo) * 180;              % Convert servo position to angle
shoulderPos = readPosition(shoulderServo) * 180;      % Convert servo position to angle
elbowPos = readPosition(elbowServo) * 180;            % Convert servo position to angle

disp(['Initial Base Servo Position: ', num2str(basePos), ' degrees']);
disp(['Initial Shoulder Servo Position: ', num2str(shoulderPos), ' degrees']);
disp(['Initial Elbow Servo Position: ', num2str(elbowPos), ' degrees']);

% Specify servo angles (in degrees)
baseAngle = 0;       % Set base servo angle to 90 degrees
shoulderAngle = 90;   % Set shoulder servo angle to 45 degrees
elbowAngle = 90;       % Set elbow servo angle to 0 degrees

% Write servo angles
writePosition(baseServo, baseAngle/180);         % Convert angle to servo position
writePosition(shoulderServo, shoulderAngle/180); % Convert angle to servo position
writePosition(elbowServo, elbowAngle/180);       % Convert angle to servo position

% Wait for servos to reach the desired positions
pause(2);

% Read and print final servo positions
basePos = readPosition(baseServo) * 180;              % Convert servo position to angle
shoulderPos = readPosition(shoulderServo) * 180;      % Convert servo position to angle
elbowPos = readPosition(elbowServo) * 180;            % Convert servo position to angle

disp(['Final Base Servo Position: ', num2str(basePos), ' degrees']);
disp(['Final Shoulder Servo Position: ', num2str(shoulderPos), ' degrees']);
disp(['Final Elbow Servo Position: ', num2str(elbowPos), ' degrees']);

% Disconnect from the Arduino
clear a;
