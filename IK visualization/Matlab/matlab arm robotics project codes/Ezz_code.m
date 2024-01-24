clc
clear all
port = 'COM6';
board = 'Mega2560';
a = arduino(port,board,'Libraries','servo');
s1 = servo(a,'D3');%base
s2 = servo(a,'D7');%shoulder
s3 = servo(a,'D12');%elbow

% Your previous code remains unchanged up to this point

% Change the target positions of the square to be in the X-Z plane
t1 = trvec2tform([x, y, z+0.03])*eul2tform([thetaz,thetay,thetax]);
t2 = trvec2tform([x-0.03, y, z+0.03])*eul2tform([thetaz,thetay,thetax]);
t3 = trvec2tform([x-0.03, y, z])*eul2tform([thetaz,thetay,thetax]);

% Your previous code remains unchanged up to this point

% Change the loop limits to the correct number of trajectory points
for i= 1:length(tvec)
    tform=tfInterp(:,:,i);
    [configSoln,solnInfo] = ik('endeffector',tform,weights,initialguess);
    thetabase=configSoln(1).JointPosition/pi;
    thetashoulder=configSoln(2).JointPosition/pi;
    thetaelbow=configSoln(3).JointPosition/pi;
    writePosition(s1,thetabase);
    writePosition(s2,thetashoulder);
    writePosition(s3,thetaelbow);
end

for i= 1:length(tvec)
    tform=tfInterp2(:,:,i);
    [configSoln,solnInfo] = ik('endeffector',tform,weights,initialguess);
    thetabase=configSoln(1).JointPosition/pi;
    thetashoulder=configSoln(2).JointPosition/pi;
    thetaelbow=configSoln(3).JointPosition/pi;
    writePosition(s1,thetabase);
    writePosition(s2,thetashoulder);
    writePosition(s3,thetaelbow);
end

for i= 1:length(tvec)
    tform=tfInterp3(:,:,i);
    [configSoln,solnInfo] = ik('endeffector',tform,weights,initialguess);
    thetabase=configSoln(1).JointPosition/pi;
    thetashoulder=configSoln(2).JointPosition/pi;
    thetaelbow=configSoln(3).JointPosition/pi;
    writePosition(s1,thetabase);
    writePosition(s2,thetashoulder);
    writePosition(s3,thetaelbow);
end

for i= 1:length(tvec)
    tform=tfInterp4(:,:,i);
    [configSoln,solnInfo] = ik('endeffector',tform,weights,initialguess);
    thetabase=configSoln(1).JointPosition/pi;
    thetashoulder=configSoln(2).JointPosition/pi;
    thetaelbow=configSoln(3).JointPosition/pi;
    writePosition(s1,thetabase);
    writePosition(s2,thetashoulder);
    writePosition(s3,thetaelbow);
end