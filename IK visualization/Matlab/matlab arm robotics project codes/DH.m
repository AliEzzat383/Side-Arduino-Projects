clc
clear all
dhparams = [0 0 1.025 0;
 0 pi/2 0.45 0;
 1.65 0 0 0;
 1.65 0 0 0;];
 %a @alp @d %th
robot = rigidBodyTree; 
body1 = rigidBody('body1');
jnt1 = rigidBodyJoint('jnt1','revolute');
setFixedTransform(jnt1,dhparams(1,:),'dh');
body1.Joint = jnt1;
addBody(robot,body1,'base')
body2 = rigidBody('body2');
jnt2 = rigidBodyJoint('jnt2','revolute');
setFixedTransform(jnt2,dhparams(2,:),'dh');
body2.Joint = jnt2
addBody(robot,body2,'body1')
body3 = rigidBody('body3');
jnt3 = rigidBodyJoint('jnt3','revolute');
setFixedTransform(jnt3,dhparams(3,:),'dh');
body3.Joint = jnt3;
addBody(robot,body3,'body2')
bodyEndEffector = rigidBody('endeffector');
setFixedTransform(bodyEndEffector.Joint,dhparams(4,:),'dh');
addBody(robot,bodyEndEffector,'body3')
config = randomConfiguration(robot)
tform= getTransform(robot,config,'endeffector','base')
ik = inverseKinematics('RigidBodyTree',robot);
weights = [0.25 0.25 0.25 1 1 1]; 
initialguess = config;
[configSoln,solnInfo] = ik('endeffector',tform,weights,initialguess);
figure(Name="PHY team")
show(robot)