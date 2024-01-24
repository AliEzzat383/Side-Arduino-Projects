% Link lengths
a1 = 100;
a2 = 70;
a3 = 50;

% Initialize variables for positions of points and angles
x1 = 0;
y1 = 0;
x2 = 0;
y2 = 0;
x3 = 0;
y3 = 0;
theta1 = 0;
theta2 = 0;
theta3 = 0;

% Screen dimensions
width = 2000;
height = 2000;

% Initialize figure
figure;
axis equal;
xlim([-1000, 1000]); % Set fixed limits for x-axis
ylim([-1000, 1000]); % Set fixed limits for y-axis
hold on;

% Main loop
running = true;
prevMousePos = [0, 0];  % Store the previous mouse position

while running
    % Clear the previous plot
    clf;

    % Get current mouse position
    mousePos = get(gca, 'CurrentPoint');
    mouseX = mousePos(1, 1);
    mouseY = mousePos(1, 2);

    % Check if mouse position changed
    if any([mouseX, mouseY] ~= prevMousePos)
        % Update the previous mouse position
        prevMousePos = [mouseX, mouseY];

        locX = mouseX - width / 2;
        locY = mouseY - height / 2;
        if locY > 0
            locY = -locY;
        end

        % Calculate inverse kinematics for mouse position
        [theta1, theta2, theta3] = inverse_kinematics_N3(a1, a2, a3, locX, locY);
        if theta1 > 0
            theta1 = -theta1;
        end
        if theta2 < -90
            theta2 = -90;
        end
        if theta3 < -90
            theta3 = -90;
        end

        % Calculate positions of points
        x1 = 0;
        y1 = 0;
        x2 = a1 * cos(deg2rad(theta1));
        y2 = a1 * sin(deg2rad(theta1));
        x3 = x2 + a2 * cos(deg2rad(theta1) + deg2rad(theta2));
        y3 = y2 + a2 * sin(deg2rad(theta1) + deg2rad(theta2));
    end

    % Draw links and joints
    plot([x1, x2, x3], [y1, y2, y3], 'o-');
    
    % Display text on plot
    text(10, -70, ['x: ', num2str(locX)]);
    text(10, -90, ['y: ', num2str(locY)]);
    text(10, 50, ['Theta1: ', num2str(theta1, '%.2f'), ' degrees']);
    text(10, 30, ['Theta2: ', num2str(theta2, '%.2f'), ' degrees']);
    text(10, 10, ['Theta3: ', num2str(theta3, '%.2f'), ' degrees']);

    % Check for key press to exit the loop
    key = get(gcf, 'CurrentCharacter');
    if key == 'q'
        running = false;
    end
    
    pause(0.1);
end

function [theta1, theta2, theta3] = inverse_kinematics_N3(L1, L2, L3, x, y)
    function eqs = equations(variables)
        theta_1 = variables(1);
        theta_2 = variables(2);
        theta_3 = variables(3);
        
        eq1 = x - L1 * cos(theta_1) - L2 * cos(theta_1 + theta_2) - L3 * cos(theta_1 + theta_2 + theta_3);
        eq2 = y - L1 * sin(theta_1) - L2 * sin(theta_1 + theta_2) - L3 * sin(theta_1 + theta_2 + theta_3);
        
        eqs = [eq1; eq2];
    end

    initial_guess = [0, 0, 0];  % Initial guess for theta_1, theta_2, and theta_3
    solution = fsolve(@equations, initial_guess);

    % Normalize angles to be within 0 to 2*pi (360 degrees)
    theta1 = mod(solution(1), 2 * pi);
    theta2 = mod(solution(2), 2 * pi);
    theta3 = mod(solution(3), 2 * pi);

    % Convert angles to degrees
    theta1 = rad2deg(theta1);
    theta2 = rad2deg(theta2);
    theta3 = rad2deg(theta3);

    % Ensure angles are in the range -180 to 180 degrees
    if theta1 > 180
        theta1 = theta1 - 360;
    end
    if theta2 > 180
        theta2 = theta2 - 360;
    end
    if theta3 > 180
        theta3 = theta3 - 360;
    end
end
