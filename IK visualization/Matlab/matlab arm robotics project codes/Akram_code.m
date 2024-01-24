clc 
clear all
% Define the kinematic model of the robotic arm
L1 = Link('d', 0.0405, 'a', 0, 'alpha', pi/2);
L2 = Link('d', 0, 'a', 0.125, 'alpha', 0);
L3 = Link('d', 0, 'a', 0.14, 'alpha', 0);
robot = SerialLink([L1 L2 L3], 'name', 'myRobot');

% Define the triangular path
p0 = [0, 0, 0.1];
p1 = [0.1, 0, 0.1];
p2 = [0.05, 0.0866, 0.1];

% Define the time interval and time vector for interpolation
tInterval = [0 1];
tvec = 0:0.01:1;

% Calculate the trajectory between the points using transformtraj
tfInterp1 = transformtraj(trvec2tform(p0), trvec2tform(p1), tInterval, tvec);
tfInterp2 = transformtraj(trvec2tform(p1), trvec2tform(p2), tInterval, tvec);
tfInterp3 = transformtraj(trvec2tform(p2), trvec2tform(p0), tInterval, tvec);

% Connect to Arduino and servos
a = arduino('COM6', 'Mega2560', 'Libraries', 'Servo');
s1 = servo(a, 'D3');    % Base servo
s2 = servo(a, 'D7');    % Shoulder servo
s3 = servo(a, 'D12');   % Elbow servo

% Define the weights and initial guess for inverse kinematics
weights = [0 0 0 1 1 1];
initialguess = zeros(1, robot.n);

% Loop over each point in the path and move the arm to that point
for i = 1:size(tfInterp1, 3)
    % Get the desired end effector position and orientation for this point
    tform = tfInterp1(:, :, i);
  
    % Use inverse kinematics to calculate the joint angles that correspond to the desired end effector position and orientation
    [configSoln, solnInfo] = robot.ikcon(tform, initialguess);
  
    % Convert the joint angles to servo positions
    thetabase = (configSoln(1)) / pi;
    thetashoulder = (configSoln(2)) / pi;
    thetaelbow = (configSoln(3)) / pi;
    % Move the servos to the desired positions
    writePosition(s1, thetabase);
    writePosition(s2, thetashoulder);
    writePosition(s3, thetaelbow);
  
    % Wait for the arm to move to the desired position
    pause(0.1);
end
