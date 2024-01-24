
theta2=0;
theta3=0; % base length 0.8M % link1 length 1.65M % link2 length 1.65M
body1 = rigidBody('body1');
jnt1 = rigidBodyJoint('jnt1','revolute');
jnt1.HomePosition = 0;
tform= trvec2tform([0, 0, 0.8])*eul2tform([0, 0, pi/2]);
setFixedTransform(jnt1,tform);
body1.Joint = jnt1;
robot = rigidBodyTree;
addBody(robot,body1,'base')
body2 = rigidBody('body2');
jnt2 = rigidBodyJoint('jnt2','revolute');
jnt2.HomePosition = 0; % User defined
tform2 = trvec2tform(1.65*cos(theta2), 1.65*sin(theta2),setFixedTransform(jnt2,tform2));
body2.Joint = jnt2;
addBody(robot,body2,'body1'); % Add body2 to body1
body3 = rigidBody('body3');
jnt3 = rigidBodyJoint('jnt3','revolute');
tform3 = trvec2tform([1.65*cos(theta3), 1.65*sin(theta3), 0]);
setFixedTransform(jnt3,tform3);
jnt3.HomePosition = 0; % User defined
body3.Joint = jnt3;
addBody(robot,body3,'body2');
bodyEndEffector = rigidBody('endeffector');
tform4 = trvec2tform(1.65*cos(theta3), 1.65*sin(theta3), 0)
setFixedTransform(bodyEndEffector.Joint,tform4);
addBody(robot,bodyEndEffector,'body3');
config = randomConfiguration(robot)
tform = getTransform(robot,config,'endeffector','base')
show(robot)