clc
clear 

a = arduino('COM3');
waist = servo(a, 'D11');
arm1 = servo(a, 'D10');
arm2 = servo(a, 'D9');

position1 = readPosition(waist); %values from 0 to 1
position2 = readPosition(arm1);
position3 = readPosition(arm2);

theta1=position1*pi;             %values converted to radian
theta2=position2*pi;
theta3=position3*pi;

body1 = rigidBody('waist');
jnt1 = rigidBodyJoint('jnt1','revolute');
jnt1.HomePosition = 0;% orientation of the waist from the base
tform = trvec2tform([0, 0, 0.1025]); % waist position
setFixedTransform(jnt1,tform);
body1.Joint = jnt1;
robot = rigidBodyTree;
addBody(robot,body1,'base')% Add waist to base
body2 = rigidBody('arm1');
jnt2 = rigidBodyJoint('jnt2','revolute');
jnt2.HomePosition = 0; % arm 1 orientation from the waist
tform2 = trvec2tform([0, 0, 0.04])*eul2tform([0, 0, pi/2]); % arm 1 position relative to the waist 
setFixedTransform(jnt2,tform2);
body2.Joint = jnt2;
addBody(robot,body2,'waist'); % Add arm1 to waist
body3 = rigidBody('arm2');
jnt3 = rigidBodyJoint('jnt3','revolute');
tform3 = trvec2tform([0.125, 0, 0])*eul2tform([0, 0, 0]); % arm 2 position relative to arm 1
setFixedTransform(jnt3,tform3);
jnt3.HomePosition = 0; % arm2 orientation from the arm1
body3.Joint = jnt3
addBody(robot,body3,'arm1'); % Add arm2 to arm1
bodyEndEffector = rigidBody('claw');
tform4 = trvec2tform([0.14, 0, 0]); % claw position
setFixedTransform(bodyEndEffector.Joint,tform4);
addBody(robot,bodyEndEffector,'arm2');
config = randomConfiguration(robot)
tform = getTransform(robot,config,'claw','base')

robot.Bodies{1, 1}.Joint.PositionLimits(1)=0;
robot.Bodies{1, 2}.Joint.PositionLimits(1)=0;
robot.Bodies{1, 3}.Joint.PositionLimits(1)=0;

%fwd%

config=randomConfiguration(robot);
config(1).JointPosition=theta1;
config(2).JointPosition=theta2;
config(3).JointPosition=theta3;
transform = getTransform(robot,config,'claw','base')

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
dist=1;
t0 = trvec2tform([x y z])*eul2tform([thetaz,thetay,thetax]); %or just saying t0=input_homogenous;  which you aquired through fwd kinematics
t1 = trvec2tform([x-dist y z])*eul2tform([thetaz,thetay,thetax]);
t2 = trvec2tform([x-dist y z+dist])*eul2tform([thetaz,thetay,thetax]);
t3 = trvec2tform([x y z+dist])*eul2tform([thetaz,thetay,thetax]);
tInterval = [0 0.5];
step=0.1;
tvec = 0:step:1;

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
%inv kinematics
ik=inverseKinematics('RigidBodyTree',robot);
% numIterations=500;
% tolerance=1e-6;
% ik.SolverParameters.MaxIterations=numIterations;
% ik.SolverParameters.SolutionTolerance=tolerance;
weights = [0 0 0 1 1 1];
initialguess = robot.homeConfiguration;
matrix_limit =(1/step)+1

for i= 1:matrix_limit
    tform=tfInterp(:,:,i);
    [configSoln,solnInfo] = ik('claw',tform,weights,initialguess);
    thetawaist=configSoln(1).JointPosition/pi
    thetaarm1=configSoln(2).JointPosition/pi
    thetaarm2=configSoln(3).JointPosition/pi
    writePosition(waist,thetawaist);
    writePosition(arm1,thetaarm1);
    writePosition(arm2,thetaarm2);

end
for i= 1:matrix_limit
    tform=tfInterp2(:,:,i);
    [configSoln,solnInfo] = ik('claw',tform,weights,initialguess);
    thetawaist=configSoln(1).JointPosition/pi
    thetaarm1=configSoln(2).JointPosition/pi
    thetaarm2=configSoln(3).JointPosition/pi
    writePosition(waist,thetawaist);
    writePosition(arm1,thetaarm1);
    writePosition(arm2,thetaarm2);
end
for i= 1:matrix_limit
    tform=tfInterp3(:,:,i);
    [configSoln,solnInfo] = ik('claw',tform,weights,initialguess);
    thetawaist=configSoln(1).JointPosition/pi
    thetaarm1=configSoln(2).JointPosition/pi
    thetaarm2=configSoln(3).JointPosition/pi
    writePosition(waist,thetawaist);
    writePosition(arm1,thetaarm1);
    writePosition(arm2,thetaarm2);

end
for i= 1:matrix_limit
    tform=tfInterp4(:,:,i);
    [configSoln,solnInfo] = ik('claw',tform,weights,initialguess);
    thetawaist=configSoln(1).JointPosition/pi
    thetaarm1=configSoln(2).JointPosition/pi
    thetaarm2=configSoln(3).JointPosition/pi
    writePosition(waist,thetawaist);
    writePosition(arm1,thetaarm1);
    writePosition(arm2,thetaarm2);
end