clear all;

% Device information
defaultFormat = 'MJPG_1280x720';
deviceFileSupported = 0;
deviceName = 'HP TrueVision HD Camera';
deviceID = 1;
videoInputConstructor = sprintf('videoinput(''winvideo'', %d, ''%s'')', deviceID, defaultFormat);
videoDeviceConstructor = sprintf('imaq.VideoDevice(''winvideo'', %d)', deviceID);
supportedFormats = {'MJPG_1280x720', 'MJPG_640x480', 'MJPG_320x240', 'YUY2_1280x720', 'YUY2_640x480', 'YUY2_320x240', 'RGB24_1280x720', 'RGB24_640x480', 'RGB24_320x240', 'GRAY8_1280x720', 'GRAY8_640x480'};

% Create video object with the desired resolution
vid = eval(videoInputConstructor);

% Set the video object to capture multiple frames
vid.FramesPerTrigger = Inf;
vid.TriggerRepeat = 0;

% Set the timeout value for getsnapshot
vid.Timeout = 120; % Set the timeout value to 120 seconds (adjust as needed)

% Create a figure to display the video stream and detected ball
figure;
axesHandle = axes('Parent', gcf);
title('White Pong Ball Detection el mafrod 0_0');

% Define the white color range in HSV color space
whiteHSVMin = [0, 0, 0.6]; % Adjust the minimum HSV values for white detection
whiteHSVMax = [1, 0.3, 1]; % Adjust the maximum HSV values for white detection

% Convert the white color range to RGB color space
whiteRGBMin = hsv2rgb(whiteHSVMin);
whiteRGBMax = hsv2rgb(whiteHSVMax);

% Set camera parameters
cameraHeight = 120  %# Camera height from the floor in centimeters
ballDiameter = 3.5 % # Diameter of the ball in centimeters (2.5 inches)
focalLength = 520  %# %Focal length of the camera in pixels
sensorHeight = 230 %# Height of the camera sensor in millimeters
sensorWidth = 7  % # Width of the camera sensor in millimeters

% Variables to store data for dumping every 5 seconds
startTimestamp = tic;
data = struct('timestamp', zeros(100, 1), 'centroid', zeros(100, 2), 'circumference', zeros(100, 1), 'position', zeros(100, 3));

% Continuously process and display the frames from the video stream
while ishandle(axesHandle)
    try
        % Read a frame from the video stream
        frame = getsnapshot(vid);

        % Convert the frame to grayscale
        frameGray = rgb2gray(frame);

        % Threshold the grayscale frame to extract white regions
        whiteMask = (frameGray >= 220); % Adjust the threshold value as needed

        % Perform morphological operations to enhance the white regions
        whiteMask = imopen(whiteMask, strel('disk', 3));
        whiteMask = imclose(whiteMask, strel('disk', 7));
        whiteMask = imfill(whiteMask, 'holes');

        % Find the centroid and bounding box of the white region
        stats = regionprops(whiteMask, 'Centroid', 'BoundingBox');
        if ~isempty(stats)
            centroid = stats.Centroid;
            boundingBox = stats.BoundingBox;

            % Calculate the circumference of the bounding box
            circumference = 2 * pi * (boundingBox(3) + boundingBox(4)) / 4;

            % Calculate the position of the ball in centimeters
            ballHeight = cameraHeight * focalLength / (centroid(2) - size(frame, 1)/2);
            ballX = (centroid(1) - size(frame, 2)/2) * ballHeight * sensorWidth / focalLength;
            ballY = ballHeight * sensorHeight / focalLength;
            ballZ = sqrt(ballHeight^2 - cameraHeight^2);
            ballPosition = [ballX, ballY, ballZ];

            % Display the grayscale frame and the detected ball
            imshow(frameGray, 'Parent', axesHandle);
            hold(axesHandle, 'on');
            viscircles(centroid, boundingBox(3)/2, 'Color', 'r', 'LineWidth', 2);
            plot(axesHandle, centroid(1), centroid(2), 'r.', 'MarkerSize', 10, 'LineWidth', 2);
            text(centroid(1), centroid(2), sprintf('Circumference: %.2f', circumference), 'Color', 'r', 'FontSize', 12, 'FontWeight', 'bold');
            text(centroid(1), centroid(2)+20, sprintf('Position: (%.2f, %.2f, %.2f) cm', ballPosition(1), ballPosition(2), ballPosition(3)), 'Color', 'r', 'FontSize', 12, 'FontWeight', 'bold');
            hold(axesHandle, 'off');

            % Store data for dumping every 5 seconds
            data(end+1).timestamp = toc(startTimestamp);
            data(end).centroid = centroid;
            data(end).circumference = circumference;
            data(end).position = ballPosition;
        else
            % Display the grayscale frame without any detection
            imshow(frameGray, 'Parent', axesHandle);
        end

        % Update the figure window
        drawnow;
    catch ME
        if strcmp(ME.identifier, 'imaqdevice:getsnapshot:interrupted')
            % Ignore the error and continue the loop
        else
            rethrow(ME);
        end
    end
    
    % Dump data every 5 seconds
    if toc(startTimestamp) >= 5
        fprintf('Dumping data:\n');
        for i = 1:numel(data)
            fprintf('Timestamp: %.2f, Centroid: (%.2f, %.2f), Circumference: %.2f, Position: (%.2f, %.2f, %.2f) cm\n', data(i).timestamp, data(i).centroid(1), data(i).centroid(2), data(i).circumference, data(i).position);
        end
        data = struct('timestamp', zeros(100, 1), 'centroid', zeros(100, 2), 'circumference', zeros(100, 1), 'position', zeros(100, 3));
        startTimestamp = tic;
    end
end

% Clean up
stop(vid);
delete(vid);
clear vid;