clc;
clear;

% DH Parameters [theta d a alpha]
DH_parameters = [0 0 1.025 0;
                 0 pi/2 0.45 0;
                 1.65 0 0 0;
                 1.65 0 0 0];

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

while true
    userInput = input('Enter x, y, and z coordinates [mm] (or type "exit" to terminate): ', 's');

    if strcmpi(userInput, 'exit')
        break; % Exit the loop
    end

    coordinates = str2double(userInput);

    if numel(coordinates) ~= 3
        disp('Invalid input. Please enter three numeric values.');
        continue; % Skip the rest of the loop and prompt again
    end

    x = coordinates(1);
    y = coordinates(2);
    z = coordinates(3);

    r = sqrt(x^2 + y^2);

    fprintf('r = %.2f\n', r);
    fprintf('x = %.2f\n', x);
    fprintf('y = %.2f\n', y);
    fprintf('z = %.2f\n', z);

    % Create a desired end effector pose
    desiredPose = robotics.Pose('Position', [x, y, z]);

    % Solve inverse kinematics
    ikSolver = robotics.InverseKinematics('RigidBodyTree', robot);
    ikSolverWeights = [0.25 0.25 0.25 1 1 1]; % Weights for the optimization
    initialGuess = robot.homeConfiguration; % Use home configuration as initial guess
    [configSoln, ~] = ikSolver(endEffectorName, desiredPose, ikSolverWeights, initialGuess);

    if isempty(configSoln)
        disp('Invalid input. The desired position is not reachable.');
        continue;
    end

    % Get the joint angles from the solution
    theta = configSoln.JointPosition;

    % Write servo angles
    writePosition(baseServo, theta(1));
    writePosition(shoulderServo, theta(2));
    writePosition(elbowServo, theta(3));
end

% Cleanup
clear a;
