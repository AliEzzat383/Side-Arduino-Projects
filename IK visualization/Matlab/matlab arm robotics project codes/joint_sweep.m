clc;
clear;

% Arduino setup
arduinoPort = 'COM6';
board = 'Mega2560';
servoLibrary = 'Servo';

% Connect to Arduino
a = arduino(arduinoPort, board, 'libraries', servoLibrary);

% Servo pins on Arduino
shoulderPin = 'D3';

% Initialize servo object
shoulderServo = servo(a, shoulderPin);

% Define shoulder sweep parameters
sweepDelay = 0.01; % Delay between each position change (in seconds)
startAngle = 0; % Starting angle (in degrees)
endAngle = 180; % Ending angle (in degrees)
stepSize = 1; % Step size for each position change (in degrees)

% Infinite loop
while true
    % Perform shoulder sweep
    for angle = startAngle:stepSize:endAngle
        % Set shoulder servo position
        writePosition(shoulderServo, angle/180);
        
        % Delay for a smooth sweep
        pause(sweepDelay);
    end
    
    % Reverse shoulder sweep
    for angle = endAngle:-stepSize:startAngle
        % Set shoulder servo position
        writePosition(shoulderServo, angle/180);
        
        % Delay for a smooth sweep
        pause(sweepDelay);
    end
end

% Cleanup
clear a;
