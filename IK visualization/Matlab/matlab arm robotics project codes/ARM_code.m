clc;
clear;

% DH Parameters [a alpha d theta]
DH_parameters = [0 0 0.1025 0;
                 0  pi/2 0.045 0;
                 0.125 0 0 0;
                 0.14 0 0 0];

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

% Create a rigid body tree object to represent the robot arm
robot = robotics.RigidBodyTree;

% Create rigid body objects for each link
body1 = robotics.RigidBody('body1');
body2 = robotics.RigidBody('body2');
body3 = robotics.RigidBody('body3');
bodyEndEffector = robotics.RigidBody('endeffector');

% Create rigid body joint objects for each joint
jnt1 = robotics.Joint('jnt1','revolute');
jnt2 = robotics.Joint('jnt2','revolute');
jnt3 = robotics.Joint('jnt3','revolute');

% Set the fixed transformations for each joint using DH parameters
jnt1.setFixedTransform(DH_parameters(1, :), 'dh');
jnt2.setFixedTransform(DH_parameters(2, :), 'dh');
jnt3.setFixedTransform(DH_parameters(3, :), 'dh');
bodyEndEffector.Joint.setFixedTransform(DH_parameters(4, :), 'dh');

% Assign joints to the corresponding bodies
body1.Joint = jnt1;
body2.Joint = jnt2;
body3.Joint = jnt3;

% Add bodies to the robot arm
addBody(robot, body1, 'base');
addBody(robot, body2, 'body1');
addBody(robot, body3, 'body2');
addBody(robot, bodyEndEffector, 'body3');

% Set the end effector name
endEffectorName = 'endeffector';

% Set joint position limits
robot.Bodies{1, 1}.Joint.PositionLimits(1) = 0;
robot.Bodies{1, 2}.Joint.PositionLimits(1) = 0;
robot.Bodies{1, 3}.Joint.PositionLimits(1) = 0;

writePosition(baseServo, 0);
writePosition(shoulderServo, 0.5);
writePosition(elbowServo, 0.5);

% Create an inverse kinematics solver
ik = robotics.InverseKinematics('RigidBodyTree', robot);
weights = [0 0 0 1 1 1];

% Generate a random initial configuration
config = randomConfiguration(robot);

% Perform inverse kinematics using the Robotics Toolbox
endEffectorPose = trvec2tform([0.1 0 0.4]);
[configSoln, ~] = ik(endEffectorName, endEffectorPose, weights, config);

% Forward Kinematics
config = randomConfiguration(robot);
config(1).JointPosition = 0;
config(2).JointPosition = 0;
config(3).JointPosition = 0;
transform = getTransform(robot, config, 'endeffector', 'base');

baseAngleRead = readPosition(baseServo) * pi;
shoulderAngleRead = readPosition(shoulderServo) * pi;
elbowAngleRead = readPosition(elbowServo) * pi;
home_config = [baseAngleRead, shoulderAngleRead, elbowAngleRead];
R = eul2rotm(deg2rad(home_config));  % Convert Euler angles to rotation matrix
B = [0.14; 0; 0.125];                % Corrected to a 3-by-1 column vector
T = eye(4);                          % Initialize 4x4 identity matrix
T(1:3, 1:3) = R;                      % Assign rotation matrix to the upper-left 3x3 submatrix
T(1:3, 4) = B;                       % Assign B as the last column
input_homogeneous = T;
inputthetas = tform2eul(input_homogeneous); % Acquire current thetas
inputposition = tform2trvec(input_homogeneous); % Acquire current position

thetaz = inputthetas(1);
thetay = inputthetas(2);
thetax = inputthetas(3);

x = inputposition(1);
y = inputposition(2);
z = inputposition(3);

% Generate desired trajectory
t0 = trvec2tform([x y z]);
t1 = trvec2tform([x y+0.2 z]);
t2 = trvec2tform([x y+0.2 z-0.2]);
t3 = trvec2tform([x y z-0.2]);

% Set joint position limits
robot.Bodies{1, 1}.Joint.PositionLimits(1) = 0;
robot.Bodies{1, 2}.Joint.PositionLimits(1) = 0;
robot.Bodies{1, 3}.Joint.PositionLimits(1) = 0;

tInterval = [0 0.4];
tvec = 0:0.01:1;

% Generate trajectory for each segment
[tfInterp, v1, a1] = transformtraj(t0, t1, tInterval, tvec);
[tfInterp2, v2, a2] = transformtraj(t1, t2, tInterval, tvec);
[tfInterp3, v3, a3] = transformtraj(t2, t3, tInterval, tvec);
[tfInterp4, v4, a4] = transformtraj(t3, t0, tInterval, tvec);

% Control the robot arm along the trajectory for each segment
trajectorySegments = {tfInterp, tfInterp2, tfInterp3, tfInterp4};
servoObjects = {baseServo, shoulderServo, elbowServo};

for segIndex = 1:numel(trajectorySegments)
    segment = trajectorySegments{segIndex};
    
    for i = 1:size(segment, 3)
        tform = segment(:,:,i);
        [configSoln, ~] = ik(endEffectorName, tform, weights, config);
        thetabase = configSoln(1).JointPosition / pi
        thetashoulder = configSoln(2).JointPosition / pi
        thetaelbow = configSoln(3).JointPosition / pi
        
        % Write servo angles
        writePosition(baseServo, 0.3343);
        writePosition(shoulderServo, 0.6657);
        writePosition(elbowServo, 1.0000);
        
        pause(0.1);
    end
end

% Read servo angles and calculate input homogeneous transformation
baseAngleRead = readPosition(baseServo) * pi;
shoulderAngleRead = readPosition(shoulderServo) * pi;
elbowAngleRead = readPosition(elbowServo) * pi;
home_config = [baseAngleRead, shoulderAngleRead, elbowAngleRead];
R = eul2rotm(deg2rad(home_config));  % Convert Euler angles to rotation matrix
B = [0.14; 0; 0.125];                % Corrected to a 3-by-1 column vector
T = eye(4);                          % Initialize 4x4 identity matrix
T(1:3, 1:3) = R;                      % Assign rotation matrix to the upper-left 3x3 submatrix
T(1:3, 4) = B;                       % Assign B as the last column
input_homogeneous = T;
inputthetas = tform2eul(input_homogeneous); % Acquire current thetas
inputposition = tform2trvec(input_homogeneous); % Acquire current position

x = inputposition(1);
y = inputposition(2);
z = inputposition(3);

t0 = trvec2tform([x y z]);

% Define the desired path (square path)
waypoints = [t0, ...
             t0 * trvec2tform([0.2 0 0]), ...
             t0 * trvec2tform([0.2 0 -0.2]), ...
             t0 * trvec2tform([0 0 -0.2]), ...
             t0];

% Generate time vector for the desired path
tvec = linspace(0, 1, size(waypoints, 3));

% Generate trajectory for the desired path
tvec = 0:0.01:1;
tfInterp = transformtraj(waypoints, linspace(0, 1, size(waypoints, 3)), tvec);

% Visualize the trajectory
rotations = tform2quat(tfInterp);
translations = tform2trvec(tfInterp);
plotTransforms(translations, rotations, 'FrameSize', 0.01);
xlabel('X');
ylabel('Y');
zlabel('Z');
hold on;

% Control the robot arm along the trajectory
for i = 1:size(tfInterp, 3)
    Tform = tfInterp(:, :, i);
    input_thetas = tform2eul(Tform);
    angle1 = rad2deg(input_thetas(1));
    angle2 = rad2deg(input_thetas(2));
    angle3 = rad2deg(input_thetas(3));
    
    % Write servo angles
    writePosition(baseServo, angle1 / 180);
    writePosition(shoulderServo, angle2 / 180);
    writePosition(elbowServo, angle3 / 180);
    
    pause(0.1);
end

% Cleanup
clear a;
