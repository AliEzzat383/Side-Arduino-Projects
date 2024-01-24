clear all;
clc;
a = arduino('COM6', 'Mega2560', 'libraries', 'Servo');

L1 = 40.5;
L2 = 165;
L3 = 165;

base = servo(a, 'D11');
shoulder = servo(a, 'D12');
arm = servo(a, 'D13');

while(1)
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

    fprintf('r = %f\n', r);
    fprintf('x = %f\n', x);
    fprintf('y = %f\n', y);
    fprintf('z = %f\n', z);

    D = (L2^2 + L3^2 - (z - L1)^2 - r^2) / (2 * L2 * L3);
    fprintf('D = %f\n', D);

    beta = atan2d(real(sqrt(1 - D^2)), D);
    fprintf('beta = %f\n', beta);
    
    if (beta < 0) 
        theta3 = beta;
        fprintf('theta3 is negative = %f\n', theta3);
    end

    if (beta > 0)
        theta3 = 180 - beta;
        fprintf('theta3 is positive = %f\n', theta3);
    end

    thetad = atan2d(y, x);
    if (thetad < 0)
        theta1 = 180 + thetad;
        fprintf('theta1 = %f\n', theta1);
    else
        theta1 = thetad;
        fprintf('theta1 = %f\n', theta1);
    end

    gamma = atan2d(z - L1, r);
    u = (L2^2 + (z - L1)^2 + r^2 - L3^2) / (2 * L2 * sqrt((z - L1)^2 + r^2));
    alpha = atan2d(real(sqrt(1 - u^2)), u);
    thetaf = gamma - alpha;
    
    if (thetaf < 0)
        theta2 = 180 + thetaf;
        fprintf('theta2 = %f\n', theta2);
    end
    if (thetaf > 0)
        theta2 = thetaf;
        fprintf('theta2 = %f\n', theta2);
    end
    
    writePosition(base, abs(theta1 / 180))
    writePosition(shoulder, abs(theta2 / 180))
    writePosition(arm, abs(theta3 / 180))
end
