clc
clear all
port = 'COM6';
board = 'Mega2560';
a = arduino(port,board,'Libraries','servo');
s1 = servo(a,'D3');%base
s2 = servo(a,'D7');%shoulder
s3 = servo(a,'D12');%elbow

position1 = readPosition(s1);
position2 = readPosition(s2);
position3 = readPosition(s3);
theta1=position1*pi;
theta2=position2*pi;
theta3=position3*pi;
body1 = rigidBody('body1');
jnt1 = rigidBodyJoint('jnt1','revolute'); jnt1.HomePosition = 0;
tform = trvec2tform([0, 0, 0.1025]); % User defined
setFixedTransform(jnt1,tform);
body1.Joint = jnt1;
robot = rigidBodyTree;
addBody(robot,body1,'base')
body2 = rigidBody('body2');
jnt2 = rigidBodyJoint('jnt2','revolute');
jnt2.HomePosition = 0 ; % User defined
tform2 = trvec2tform([0, 0, 0.045])*eul2tform([0, 0, pi/2]); % User defined
setFixedTransform(jnt2,tform2);
body2.Joint = jnt2;
addBody(robot,body2,'body1'); % Add body2 to body1
body3 = rigidBody('body3');
jnt3 = rigidBodyJoint('jnt3','revolute');
tform3 = trvec2tform([0.125, 0, 0]); % User defined
setFixedTransform(jnt3,tform3);
jnt3.HomePosition = 0 ; % User defined
body3.Joint = jnt3
addBody(robot,body3,'body2'); % Add body3 to body2
bodyEndEffector = rigidBody('endeffector');
tform5 = trvec2tform([0.14, 0, 0]); % User defined
setFixedTransform(bodyEndEffector.Joint,tform5);
addBody(robot,bodyEndEffector,'body3');
robot.Bodies{1, 1}.Joint.PositionLimits(1)=0;
robot.Bodies{1, 2}.Joint.PositionLimits(1)=0;
robot.Bodies{1, 3}.Joint.PositionLimits(1)=0;

%fwd%
config=randomConfiguration(robot);
config(1).JointPosition=theta1;
config(2).JointPosition=theta2;
config(3).JointPosition=theta3;
transform = getTransform(robot,config,'endeffector','base')
% input to the matrices
input_homogenous=transform %example for a homogeneos matrix, INSTEAD take the tform of your current random position.

inputthetas = tform2eul(input_homogenous); %aquiring current thetas
inputposition = tform2trvec(input_homogenous);%aquiring current position


thetaz=inputthetas(1);
thetay=inputthetas(2);
thetax=inputthetas(3);

x=inputposition(1);
y=inputposition(2);
z=inputposition(3);


t0 = trvec2tform([x y z])*eul2tform([thetaz,thetay,thetax]); %or just saying t0=input_homogenous;  which you aqured through fwd kinematics
t1 = trvec2tform([x-0.03 y z])*eul2tform([thetaz,thetay,thetax]);
t2 = trvec2tform([x-0.03 y z+0.03])*eul2tform([thetaz,thetay,thetax]);
t3 = trvec2tform([x y z+0.03])*eul2tform([thetaz,thetay,thetax]);
tInterval = [0 0.05];
tvec = 0:0.01:1;

[tfInterp, v1, a1] = transformtraj(t0,t1,tInterval,tvec);


% the following lines of codes prints the frames in a figure that helps
% visualizing the path
% rotations = tform2quat(tfInterp);
% translations = tform2trvec(tfInterp);
% plotTransforms(translations,rotations)
% xlabel('X')
% ylabel('Y')
% zlabel('Z')
% hold on;



[tfInterp2, v1, a1] = transformtraj(t1,t2,tInterval,tvec);

% rotations2 = tform2quat(tfInterp2);
% translations2 = tform2trvec(tfInterp2);
% plotTransforms(translations2,rotations2,'FrameSize',0.01)
% hold on;

[tfInterp3, v1, a1] = transformtraj(t2,t3,tInterval,tvec);
% 
% rotations3 = tform2quat(tfInterp3);
% translations3 = tform2trvec(tfInterp3);
% plotTransforms(translations3,rotations3,'FrameSize',0.01)


[tfInterp4, v1, a1] = transformtraj(t3,t0,tInterval,tvec);
% 
% rotations4 = tform2quat(tfInterp4);
% translations4 = tform2trvec(tfInterp4);
% plotTransforms(translations4,rotations4,'FrameSize',0.01)


%in the end of this code you have 4 array of matrices that defines a
%square path

%to access any array use this example
% Tform5 = tfInterp(:,:,5)
%inv%
ik=inverseKinematics('RigidBodyTree',robot);
% numIterations=500;
% tolerance=1e-6;
% ik.SolverParameters.MaxIterations=numIterations;
% ik.SolverParameters.SolutionTolerance=tolerance;
weights = [0 0 0 1 1 1];
initialguess = robot.homeConfiguration;
for i= 1:10
    tform=tfInterp(:,:,i);
    [configSoln,solnInfo] = ik('endeffector',tform,weights,initialguess);
    thetabase=configSoln(1).JointPosition/pi
    thetashoulder=configSoln(2).JointPosition/pi
    thetaelbow=configSoln(3).JointPosition/pi
    writePosition(s1,configSoln(1).JointPosition/pi);
    writePosition(s2,configSoln(2).JointPosition/pi);
    writePosition(s3,configSoln(3).JointPosition/pi);

end
for i= 1:10
    tform=tfInterp2(:,:,i);
    [configSoln,solnInfo] = ik('endeffector',tform,weights,initialguess);
    thetabase=configSoln(1).JointPosition/pi
    thetashoulder=configSoln(2).JointPosition/pi
    thetaelbow=configSoln(3).JointPosition/pi
    writePosition(s1,configSoln(1).JointPosition/pi);
    writePosition(s2,configSoln(2).JointPosition/pi);
    writePosition(s3,configSoln(3).JointPosition/pi);

end
for i= 1:10
    tform=tfInterp3(:,:,i);
    [configSoln,solnInfo] = ik('endeffector',tform,weights,initialguess);
    thetabase=configSoln(1).JointPosition/pi
    thetashoulder=configSoln(2).JointPosition/pi
    thetaelbow=configSoln(3).JointPosition/pi
    writePosition(s1,configSoln(1).JointPosition/pi);
    writePosition(s2,configSoln(2).JointPosition/pi);
    writePosition(s3,configSoln(3).JointPosition/pi);

end
for i= 1:10
    tform=tfInterp4(:,:,i);
    [configSoln,solnInfo] = ik('endeffector',tform,weights,initialguess);
    thetabase=configSoln(1).JointPosition/pi
    thetashoulder=configSoln(2).JointPosition/pi
    thetaelbow=configSoln(3).JointPosition/pi
    writePosition(s1,configSoln(1).JointPosition/pi);
    writePosition(s2,configSoln(2).JointPosition/pi);
    writePosition(s3,configSoln(3).JointPosition/pi);

end