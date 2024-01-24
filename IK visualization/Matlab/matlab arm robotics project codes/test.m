clc;
clear;

% DH Parameters [a alpha d theta]
DH_parameters = [0 0 0.1025 0;
                 0 pi/2 0.0405 0;
                 0.165 0 0 0;
                 0.165 0 0 0];

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
%assigning joints homing positions
jnt1.HomePosition =  readPosition(baseServo);
jnt2.HomePosition = readPosition(shoulderServo);
jnt3.HomePosition =readPosition(elbowServo);
%defining joint to joint transformations

% Add bodies to the robot arm
addBody(robot, body1, 'base');
addBody(robot, body2, 'body1');
addBody(robot, body3, 'body2');
addBody(robot, bodyEndEffector, 'body3');

% Set the end effector name
endEffectorName = 'endeffector';

% Set joint position limits
robot.Bodies{1, 1}.Joint.PositionLimits = [0 0];
robot.Bodies{1, 2}.Joint.PositionLimits = [0 0];
robot.Bodies{1, 3}.Joint.PositionLimits = [0 0];

% Create an inverse kinematics solver
ik = robotics.InverseKinematics('RigidBodyTree', robot);
weights = [0 0 0 1 1 1];


initialguess = robot.homeConfiguration;
% Define the desired end effector poses
x = 0;   % x-coordinate
y = 0;   % y-coordinate
z = 4; % z-coordinate
% tr2vec
% 
% thx=0;
% thy=0.35;
% thz=0.35;


% confignew(1).JointPosition=thx;

tform=getransform()


while true
    % Read servo angles
    baseAngle = readPosition(baseServo)*pi;
    shoulderAngle = readPosition(shoulderServo)*pi;
    elbowAngle = readPosition(elbowServo)*pi;

    % Create the transformation matrix (tform)
    initial = eye(4);
    initial(1:3, 4) = [x; y; z];
    initial(1:3, 1:3) = eul2rotm([0, shoulderAngle, elbowAngle]);

    % Display the transformation matrix
    disp('Initial Transformation Matrix:');
    disp(initial);

    % Perform inverse kinematics using the Robotics Toolbox
    endEffectorPose = initial;
    [config,solninfo] = ik(endEffectorName, endEffectorPose, weights,initialguess);
[configSoln,solnInfo] = ik('L6',tform,weights,initialguess);
    % Print joint angles
%     fprintf('Base angle: %.2f\n', config(1));
%     fprintf('Shoulder angle: %.2f\n', config(2));
%     fprintf('Elbow angle: %.2f\n', config(3));

    % Write servo angles
    writePosition(baseServo, config(1).JointPosition);
    writePosition(shoulderServo, config(2).JointPosition);
    writePosition(elbowServo, config(3).JointPosition);
end

% Cleanup
clear a;
