clc;
clear;

% Arduino setup
arduinoPort = 'COM6';
board = 'Mega2560';
servoLibrary = 'Servo';

% Connect to Arduino
a = arduino(arduinoPort, board, 'libraries', servoLibrary);

% Servo pins on Arduino
basePin = 'D3';
shoulderPin = 'D7';
elbowPin = 'D12';

% Initialize servo objects
baseServo = servo(a, basePin);
shoulderServo = servo(a, shoulderPin);
elbowServo = servo(a, elbowPin);

% Desired end effector position
x = 5;      % Desired x-coordinate (in cm)
y = 0;      % Desired y-coordinate (in cm)
z = 4;   % Desired z-coordinate (in cm)

% Link lengths
L1 = 12.5;  % Length of link 1 (in cm)
L2 = 14;    % Length of link 2 (in cm)

% Perform inverse kinematics calculations
theta1 = atan2d(y, x);
D = (x^2 + y^2 + z^2 - L1^2 - L2^2) / (2 * L1 * L2);
theta2 = atan2d(sqrt(1 - D^2), D) + atan2d(z, sqrt(x^2 + y^2));
theta3 = atan2d(z, sqrt(x^2 + y^2)) - theta2;
if theta3 < 0
    theta3=abs(theta3);
end
% Display the joint angles
fprintf('Joint Angles:\n');
fprintf('theta1: %.2f degrees\n', theta1);
fprintf('theta2: %.2f degrees\n', theta2);
fprintf('theta3: %.2f degrees\n', theta3);
baseAngleRead = readPosition(baseServo) * 180;
shoulderAngleRead = readPosition(shoulderServo) * 180;
elbowAngleRead = readPosition(elbowServo) * 180; 


% Move the servos to the desired positions
writePosition(baseServo, theta1/180);
writePosition(shoulderServo, theta2/180);
writePosition(elbowServo, theta3/180);

fprintf('Read Angles:\n');
fprintf('theta1: %.2f degrees\n', baseAngleRead);
fprintf('theta2: %.2f degrees\n', shoulderAngleRead);
fprintf('theta3: %.2f degrees\n', elbowAngleRead);

% Cleanup
clear a;
