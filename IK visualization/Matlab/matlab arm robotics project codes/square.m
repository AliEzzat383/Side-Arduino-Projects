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

% Link lengths
L1 = 12.5;  % Length of link 1 (in cm)
L2 = 14;    % Length of link 2 (in cm)

% Square pattern parameters
sideLength = 10;  % Side length of the square (in cm)
numSides = 4;     % Number of sides in the square

% Perform inverse kinematics and draw the square pattern
for i = 1:numSides
    % Desired end effector position for the current side
    x = sideLength;
    y = 0;
    z = 0;

    % Perform inverse kinematics calculations
    theta1 = atan2d(y, x);
    D = (x^2 + y^2 + z^2 - L1^2 - L2^2) / (2 * L1 * L2);
    theta2 = atan2d(sqrt(1 - D^2), D) + atan2d(z, sqrt(x^2 + y^2));
    theta3 = atan2d(z, sqrt(x^2 + y^2)) - theta2;
    if theta3 < 0
        theta3 = abs(theta3);
    end
    
    % Display the joint angles
    fprintf('Joint Angles:\n');
    fprintf('theta1: %.2f degrees\n', theta1);
    fprintf('theta2: %.2f degrees\n', theta2);
    fprintf('theta3: %.2f degrees\n', theta3);

    % Move the servos to the desired positions
    writePosition(baseServo, theta1/180);
    writePosition(shoulderServo, theta2/180);
    writePosition(elbowServo, theta3/180);
    
    % Wait for the arm to reach the desired position
    pause(1);
    
    % Update the end effector position for the next side
    x = 0;
    y = sideLength;
    z = 0;
    
    % Perform inverse kinematics calculations
    theta1 = atan2d(y, x);
    D = (x^2 + y^2 + z^2 - L1^2 - L2^2) / (2 * L1 * L2);
    theta2 = atan2d(sqrt(1 - D^2), D) + atan2d(z, sqrt(x^2 + y^2));
    theta3 = atan2d(z, sqrt(x^2 + y^2)) - theta2;
    if theta3 < 0
        theta3 = abs(theta3);
    end
    
    % Display the joint angles
    fprintf('Joint Angles:\n');
    fprintf('theta1: %.2f degrees\n', theta1);
    fprintf('theta2: %.2f degrees\n', theta2);
    fprintf('theta3: %.2f degrees\n', theta3);

    % Move the servos to the desired positions
    writePosition(baseServo, theta1/180);
    writePosition(shoulderServo, theta2/180);
    writePosition(elbowServo, theta3/180);
    
    % Wait for the arm to reach the desired position
    pause(1);
end

% Cleanup
clear a;
