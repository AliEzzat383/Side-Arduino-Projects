% input to the matrices
input_homogenous=[1 0 0 0;0 1 0 0;0 0 -1 0;0 0 0 1] %example for a homogeneos matrix, INSTEAD take the tform of your current random position.


inputthetas = tform2eul(input_homogenous); %aquiring current thetas
inputposition = tform2trvec(input_homogenous);%aquiring current position


thetax=inputthetas(1);
thetay=inputthetas(2);
thetaz=inputthetas(3);

x=inputposition(1);
y=inputposition(2);
z=inputposition(3);


t0 = trvec2tform([x y z])*eul2tform([thetax,thetay,thetaz]); %or just saying t0=input_homogenous;  which you aqured through fwd kinematics
t1 = trvec2tform([x+0.2 y z]);
t2 = trvec2tform([x+0.2 y z-0.2]);
t3 = trvec2tform([x y z-0.2]);

tInterval = [0 1];
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

% rotations3 = tform2quat(tfInterp3);
% translations3 = tform2trvec(tfInterp3);
% plotTransforms(translations3,rotations3,'FrameSize',0.01)


[tfInterp4, v1, a1] = transformtraj(t3,t0,tInterval,tvec);

% rotations4 = tform2quat(tfInterp4);
% translations4 = tform2trvec(tfInterp4);
% plotTransforms(translations4,rotations4,'FrameSize',0.01)


%in the end of this code you have 4 array of matrices that defines a
%square path

%to access any array use this example
Tform5 = tfInterp(:,:,5)
