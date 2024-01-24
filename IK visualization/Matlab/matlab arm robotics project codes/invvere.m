clear all;
clc;
a = arduino('COM6', 'Mega2560', 'libraries', 'Servo');

l1 = 102.5;
l2 = 165;
l3 = 165;

base = servo(a, 'D3');
shoulder = servo(a, 'D7');
arm = servo(a, 'D12');

while true
    userInput = input('Enter x, y, and z coordinates [mm] (or type "exit" to terminate): ', 's');

    if strcmpi(userInput, 'exit')
        break; % Exit the loop
    end

    x = str2double(userInput);

    if isnan(x)
        disp('Invalid input. Please enter numeric values.');
        continue; % Skip the rest of the loop and prompt again
    end

    y = input('Enter a number: ');
    z = input('Enter a number: ');

    r = sqrt(x^2 + y^2);
    
    fprintf('r = %.2f\n', r);
    fprintf('x = %.2f\n', x);
    fprintf('y = %.2f\n', y);
    fprintf('z = %.2f\n', z);

    D = (l2^2 + l3^2 - (z - l1)^2 - r^2) / (2 * l2 * l3);
    beta = atan2d(real(sqrt(1 - D^2)), D);
    fprintf('beta = %.2f\n', beta);

    if beta < 0
        theta3 = beta;
        fprintf('theta3 = %.2f\n', theta3);
    else
        theta3 = 180 - beta;
        fprintf('theta3 = %.2f\n', theta3);
    end

    thetad = atan2d(y, x);

    if thetad < 0
        theta1 = 180 + thetad;
        fprintf('theta1 = %.2f\n', theta1);
    else
        theta1 = thetad;
        fprintf('theta1 = %.2f\n', theta1);
    end

    gamma = atan2d(z - l1, r);
    u = (l2^2 + (z - l1)^2 + r^2 - l3^2) / (2 * l2 * sqrt((z - l1)^2 + r^2));
    alpha = atan2d(real(sqrt(1 - u^2)), u);
    thetaf = gamma - alpha;

    if thetaf < 0
        theta2 = 180 + thetaf;
        fprintf('theta2 = %.2f\n', theta2);
    else
        theta2 = thetaf;
        fprintf('theta2 = %.2f\n', theta2);
    end

    writePosition(base, abs(theta1/180));
    writePosition(shoulder, abs(theta2/180));
    writePosition(arm, abs(theta3/180));
end

% Cleanup
clear a;
