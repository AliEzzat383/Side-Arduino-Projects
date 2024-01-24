clc
clear all;
a = arduino('COM6', 'Mega2560', 'libraries', 'Servo');

L1 = 0.045;
L2 = 0.125;
L3 = 0.140;

base = servo(a, 'D3');
shoulder = servo(a, 'D7');
arm = servo(a, 'D12');

while(1)
    % Read voltage values from analog inputs
    volt = readVoltage(a, 'A0');
    angle1 = volt * 180 / 5;

    volt = readVoltage(a, 'A1');
    angle2 = volt * 180 / 5;

    volt = readVoltage(a, 'A2');
    angle3 = volt * 180 / 5;

    r = L2 * cosd(angle2) + L3 * cosd(angle2 + angle3);
    z = L2 * sind(angle2) + L3 * sind(angle2 + angle3) + L1;
    x = r * cosd(angle1);
    y = r * sind(angle1);

    % Define DH parameters
    dhparams = [0 0 0.1025 0;
                0 pi/2 0.045 0;
                0.125 0 0 0;
                0.14 0 0 0];

    % Create a rigid body tree
    robot = rigidBodyTree;
    body1 = rigidBody('body1');
    jnt1 = rigidBodyJoint('jnt1','revolute');
    setFixedTransform(jnt1, dhparams(1,:), 'dh');
    body1.Joint = jnt1;
    addBody(robot, body1, 'base');

    body2 = rigidBody('body2');
    jnt2 = rigidBodyJoint('jnt2','revolute');
    setFixedTransform(jnt2, dhparams(2,:), 'dh');
    body2.Joint = jnt2;
    addBody(robot, body2, 'body1');

    body3 = rigidBody('body3');
    jnt3 = rigidBodyJoint('jnt3','revolute');
    setFixedTransform(jnt3, dhparams(3,:), 'dh');
    body3.Joint = jnt3;
    addBody(robot, body3, 'body2');

    bodyEndEffector = rigidBody('endeffector');
    setFixedTransform(bodyEndEffector.Joint, dhparams(4,:), 'dh');
    addBody(robot, bodyEndEffector, 'body3');

    config = homeConfiguration(robot);
    tform = getTransform(robot, config, 'endeffector', 'base');

    a = [x, y, z, 1];
    b = transpose(a);
    tform1 = tform .* b;
    ik = inverseKinematics('RigidBodyTree', robot);
    weights = [0 0 0 1 1 1];
    initialguess = config;
    [configSoln, solnInfo] = ik('endeffector', tform1, weights, initialguess);

    input_homogenous = tform1;
    inputthetas = tform2eul(input_homogenous); % Acquire current thetas
    inputposition = tform2trvec(input_homogenous); % Acquire current position

    thetax = inputthetas(1);
    thetay = inputthetas(2);
    thetaz = inputthetas(3);

    t0 = trvec2tform([x y z]) * eul2tform([thetax, thetay, thetaz]);
    t1 = trvec2tform([x+0.2 y z]) * eul2tform([thetax, thetay, thetaz]);
    t2 = trvec2tform([x+0.2 y z-0.2]) * eul2tform([thetax, thetay, thetaz]);
    t3 = trvec2tform([x y z-0.2]) * eul2tform([thetax, thetay, thetaz]);

    tInterval = [0 1];
    tvec = 0:0.01:1;

    [tfInterp, v1, a1] = transformtraj(t0, t1, tInterval, tvec);

    rotations = tform2quat(tfInterp);
    translations = tform2trvec(tfInterp);
    plotTransforms(translations, rotations)
    xlabel('X')
    ylabel('Y')
    zlabel('Z')
    hold on;

    [tfInterp2, v2, a2] = transformtraj(t1, t2, tInterval, tvec);

    rotations2 = tform2quat(tfInterp2);
    translations2 = tform2trvec(tfInterp2);
    plotTransforms(translations2, rotations2, 'FrameSize', 0.01)
    hold on;

    [tfInterp3, v3, a3] = transformtraj(t2, t3, tInterval, tvec);

    rotations3 = tform2quat(tfInterp3);
    translations3 = tform2trvec(tfInterp3);
    plotTransforms(translations3, rotations3, 'FrameSize', 0.01)

    [tfInterp4, v4, a4] = transformtraj(t3, t0, tInterval, tvec);

    rotations4 = tform2quat(tfInterp4);
    translations4 = tform2trvec(tfInterp4);
    plotTransforms(translations4, rotations4, 'FrameSize', 0.01)

    for m = 1:101
        Tform5 = tfInterp(:,:,m);
        inputthetas = tform2eul(Tform5);

        thetax = inputthetas(1);
        thetay = inputthetas(2);
        thetaz = inputthetas(3);
        angle15 = abs(thetax * (180 / pi));
        angle25 = abs(thetay * (180 / pi));
        angle35 = abs(thetaz * (180 / pi));
        writePosition(base, angle15/180);
        writePosition(shoulder, angle25/180);
        writePosition(arm, angle35/180);
        pause(0.1);
    end

    for o = 1:101
        Tform6 = tfInterp2(:,:,o);
        inputthetas1 = tform2eul(Tform6);
        thetax = inputthetas1(1);
        thetay = inputthetas1(2);
        thetaz = inputthetas1(3);
        angle16 = abs(thetax * (180 / pi));
        angle26 = abs(thetay * (180 / pi));
        angle36 = abs(thetaz * (180 / pi));
        writePosition(base, angle16/180);
        writePosition(shoulder, angle26/180);
        writePosition(arm, angle36/180);
        pause(0.1);
    end

    for a = 1:101
        Tform7 = tfInterp3(:,:,a);
        inputthetas2 = tform2eul(Tform7);

        thetax = inputthetas2(1);
        thetay = inputthetas2(2);
        thetaz = inputthetas2(3);
        angle17 = abs(thetax * (180 / pi));
        angle27 = abs(thetay * (180 / pi));
        angle37 = abs(thetaz * (180 / pi));
        writePosition(base, angle17/180);
        writePosition(shoulder, angle27/180);
        writePosition(arm, angle37/180);
        pause(0.1);
    end

    for t = 1:101
        Tform8 = tfInterp4(:,:,t);
        inputthetas3 = tform2eul(Tform8);
        thetax = inputthetas3(1);
        thetay = inputthetas3(2);
        thetaz = inputthetas3(3);
        angle18 = abs(thetax * (180 / pi));
        angle28 = abs(thetay * (180 / pi));
        angle38 = abs(thetaz * (180 / pi));
        writePosition(base, angle18/180);
        writePosition(shoulder, angle28/180);
        writePosition(arm, angle38/180);
        pause(0.1);
    end
end
