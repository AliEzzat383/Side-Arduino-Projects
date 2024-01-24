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

% Set step size and delay
stepSize = -1; % Adjust step size as needed
sweepDelay = 0.2; % Adjust sweep delay as needed
startAngle = 160; % Starting angle in degrees
endAngle = 180; % Ending angle in degrees

% Move servos to initial position
writePosition(baseServo, 0);
writePosition(shoulderServo, 150);
writePosition(elbowServo, 0);

% Sweep the shoulder servo from start angle to end angle
for angle = startAngle:stepSize:endAngle
    % Set shoulder servo position
    writePosition(shoulderServo, angle/180);
    
    % Delay for a smooth sweep
    pause(sweepDelay);
end

% Cleanup
clear a;
