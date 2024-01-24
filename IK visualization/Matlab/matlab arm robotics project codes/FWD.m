clc;
clear;

% Arduino setup
arduinoPort = 'COM6';
board = 'Mega2560';
servoLibrary = 'Servo';

% Connect to Arduino
a = arduino(arduinoPort, board, 'libraries', servoLibrary);

% Servo pins on Arduino
shoulderPin = 'D7';
elbowPin = 'D12';

% Initialize servo objects
shoulderServo = servo(a, shoulderPin);
elbowServo = servo(a, elbowPin);

% Joint angles (theta1 and theta2)
theta1_deg = 90;  % Angle of joint 1 (in degrees)
theta2_deg = 0;   % Angle of joint 2 (in degrees)

% Convert joint angles to radians
theta1_rad = deg2rad(theta1_deg);
theta2_rad = deg2rad(theta2_deg);

% Link lengths
L1 = 12.5;  % Length of link 1 (in cm)
L2 = 14;    % Length of link 2 (in cm)

% Forward kinematics calculations
x = L1 * cos(theta1_rad) + L2 * cos(theta1_rad + theta2_rad);
y = L1 * sin(theta1_rad) + L2 * sin(theta1_rad + theta2_rad);

% Display the end effector position
fprintf('End Effector Position:\n');
fprintf('x: %.2f cm\n', x);
fprintf('y: %.2f cm\n', y);

% Set the servo angles
shoulderAngle = mapToServo(theta1_deg, 0, 180);  % Replace mapToServo with your calibration function
elbowAngle = mapToServo(theta2_deg, 0, 180);     % Replace mapToServo with your calibration function
writePosition(shoulderServo, shoulderAngle/180);
writePosition(elbowServo, elbowAngle);

% Cleanup
clear a;

% Custom calibration function to map joint angles to servo positions
function servoPosition = mapToServo(angle, angleMin, angleMax)
    servoMin = 0;    % Minimum servo position
    servoMax = 180;  % Maximum servo position

    % Map the input angle to the servo position range
    servoPosition = map(angle, angleMin, angleMax, servoMin, servoMax);
end

% Custom map function implementation
function y = map(x, inMin, inMax, outMin, outMax)
    y = (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
end
