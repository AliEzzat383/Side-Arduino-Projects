clear a
clear all;
clc;
a =arduino('COM6' ,'Mega2560', 'libraries', 'Servo');

l1 = 40.5  ;
l2 = 12.5;
l3 = 14;
s1 = servo(a, 'D3');
s2 = servo(a, 'D7');
s3 = servo(a, 'D12');
% angle1 = readPosition(s1)*180
% angle2 = readPosition(s2)*180
% angle3 = readPosition(s3)*180


theta1=0
theta2=0;
theta3=0;

body1 = rigidBody('body1');
jnt1 = rigidBodyJoint('jnt1','revolute');
jnt1.HomePosition = 0;
 
tform= trvec2tform([0, 0, 10.25])*eul2tform([0, 0, theta1]); % User defined
setFixedTransform(jnt1,tform);
body1.Joint = jnt1; 
 
robot = rigidBodyTree;
 
addBody(robot,body1,'base');
body2 = rigidBody('body2');
jnt2 = rigidBodyJoint('jnt2','revolute');
jnt2.HomePosition = 0; 
tform2 = trvec2tform([0, 4.5, 0])*eul2tform([0, 0, theta2]); 
setFixedTransform(jnt2,tform2);
body2.Joint = jnt2;
addBody(robot,body2,'body1'); 
 
body3 = rigidBody('body3');
jnt3 = rigidBodyJoint('jnt3','revolute');
tform3 = trvec2tform([16.5*cos(theta3), 16.5*sin(theta3), 0])
setFixedTransform(jnt3,tform3);
jnt3.HomePosition = 0; % User defined
body3.Joint = jnt3
addBody(robot,body3,'body2');
 
bodyEndEffector = rigidBody('endeffector');
tform4 = trvec2tform([16.5*cos(theta3), 16.5*sin(theta3), 0]); 
setFixedTransform(bodyEndEffector.Joint,tform4);
addBody(robot,bodyEndEffector,'body3');
 
config = randomConfiguration(robot)
tform = getTransform(robot,config,'endeffector','base')

% Set the initial position and orientation of the robotic arm
input_homogenous =tform 
inputthetas = tform2eul(input_homogenous);
inputposition = tform2trvec(input_homogenous);
thetax = inputthetas(1);
thetay = inputthetas(2);
thetaz = inputthetas(3); 
x = inputposition(1);
y = inputposition(2);
z = inputposition(3);
%%%%%%%%%%%%%%%%%%%%%%
while(1)
t0 = trvec2tform([x y z]) * eul2tform([thetax, thetay, thetaz]);
t1 = trvec2tform([x, y, z+50])* eul2tform([thetax, thetay, thetaz]);
t2 = trvec2tform([x+50, y, z+50])* eul2tform([thetax, thetay, thetaz]);
t3 = trvec2tform([x+50, y, z])* eul2tform([thetax, thetay, thetaz]);
tInterval = [0 1];
tvec = 0:0.1:1;

[tfInterp, ~, ~] = transformtraj(t0, t1, tInterval, tvec);
[tfInterp2, ~, ~] = transformtraj(t1, t2, tInterval, tvec);
[tfInterp3, ~, ~] = transformtraj(t2, t3, tInterval, tvec);
[tfInterp4, ~, ~] = transformtraj(t3, t0, tInterval, tvec);



for i = 1:length(tvec)
    
    Tform = tfInterp(:, :, i);
    
inputposition = tform2trvec(Tform)
x = inputposition(1);
y = inputposition(2);
z = inputposition(3);
r= sqrt(x^2+y^2)
    s=z-l1;
    w=sqrt(s^2 + r^2)
    D = (w^2 - l2^2 - l3^2) / (2 * l2 * l3);%d=-0.6427876101
    %D=abs(d)

    theta3 = atan2d(real(sqrt(1 - D^2)), D)
    
    theta1 = atan2d(y,x)
    
    gamma = atan2d(z - l1, r)
    alpha=atan2d(l3*sind(theta3),l2+l3*cosd(theta3))
    theta2 = gamma - alpha

theta1=((theta1+180)/360)
theta2=((theta2+180)/360)
theta3=((theta3+180)/360)
writePosition(s1,theta1)
writePosition(s2,theta2)
writePosition(s3,theta3)
pause(0.1);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for i = 1:length(tvec)
    
    Tform = tfInterp2(:, :, i);
    
inputposition = tform2trvec(Tform)
x = inputposition(1);
y = inputposition(2);
z = inputposition(3);
r= sqrt(x^2+y^2)
    s=z-l1;
    w=sqrt(s^2 + r^2)
    D = (w^2 - l2^2 - l3^2) / (2 * l2 * l3);%d=-0.6427876101
    %D=abs(d)

    theta3 = atan2d(real(sqrt(1 - D^2)), D)
    
    theta1 = atan2d(y,x)
    
    gamma = atan2d(z - l1, r);
    alpha=atan2d(l3*sind(theta3),l2+l3*cosd(theta3))
    theta2 = gamma - alpha

theta1=((theta1+180)/360)
theta2=((theta2+180)/360)
theta3=((theta3+180)/360)
writePosition(s1,theta1)
writePosition(s2,theta2)
writePosition(s3,theta3)
pause(0.1);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for i = 1:length(tvec)
    
    Tform = tfInterp3(:, :, i);
    
inputposition = tform2trvec(Tform)
x = inputposition(1);
y = inputposition(2);
z = inputposition(3);
i
r= sqrt(x^2+y^2)
    s=z-l1;
    w=sqrt(s^2 + r^2)
    D = (w^2 - l2^2 - l3^2) / (2 * l2 * l3);%d=-0.6427876101
    %D=abs(d)

    theta3 = atan2d(real(sqrt(1 - D^2)), D)
    
    theta1 = atan2d(y,x)
    
    gamma = atan2d(z - l1, r);
    alpha=atan2d(l3*sind(theta3),l2+l3*cosd(theta3))
    theta2 = gamma - alpha

    theta1=((theta1+180)/360)
theta2=((theta2+180)/360)
theta3=((theta3+180)/360)
writePosition(s1,theta1)
writePosition(s2,theta2)
writePosition(s3,theta3)
  %  pause(0.1);
end


for i = 1:length(tvec)
    
    Tform = tfInterp4(:, :, i);
    
inputposition = tform2trvec(Tform)
x = inputposition(1);
y = inputposition(2);
z = inputposition(3);
r= sqrt(x^2+y^2)
i
    s=z-l1;
    w=sqrt(s^2 + r^2)
    D = (w^2 - l2^2 - l3^2) / (2 * l2 * l3);%d=-0.6427876101
    %D=abs(d)

    theta3 = atan2d(real(sqrt(1 - D^2)), D)
    
    theta1 = atan2d(y,x)
    
    gamma = atan2d(z - l1, r);
    alpha=atan2d(l3*sind(theta3),l2+l3*cosd(theta3))
    theta2 = gamma - alpha

    theta1=((theta1+180)/360)
theta2=((theta2+180)/360)
theta3=((theta3+180)/360)
writePosition(s1,theta1)
writePosition(s2,theta2)
writePosition(s3,theta3)
   % pause(0.1);
end
end