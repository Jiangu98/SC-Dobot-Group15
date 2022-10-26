% Group 15 Project - Control and Grasping for DoBot Robot
% roscore
% cd /usr/local/MATLAB/R2020a/bin 
% roslaunch dobot_magician_driver dobot_magician.launch
% roslaunch realsense2_camera rs_camera.launch 
% https://github.com/gapaul/dobot_magician_driver/wiki/MATLAB-Example

% rosbag - http://wiki.ros.org/rosbag/Commandline OR http://wiki.ros.org/rosbag/Tutorials
% cd liam/405C-0F7C/SnC/SC-Dobot-Group15/Main\ scripts/Bags/ 
% rostopic list -v 
% rosbag record -O Projectrosbag1.bag usb_cam/image_raw --duration=5 

% 1. Initialise Dobot and ROS
% 2. Camera Calibration
% 3. Detect coloured cubes
% 4. Sort coloured cubes based off colour

%%
clc
clf
clear all

%% Setup and Initialisation 
rosshutdown;
rosinit;                    % If driver is on your laptop
% rosinit(ip address);      % If connecting to UTS ip
% startup_rvc;
pause(1);

%% Set initialise status for Dobot
[safetyStatePublisher,safetyStateMsg] = rospublisher('/dobot_magician/target_safety_status');
safetyStateMsg.Data = 2;
send(safetyStatePublisher,safetyStateMsg);
pause(24);
% CurrentDobotStatus = rossubscriber('/dobot_magician/target_safety_status');
% DobotStatus = CurrentDobotStatus.LatestMessage.Data

%% Make Default Joint state
% Set target joint state
jointTarget = deg2rad([0,0,0,-45]); % Remember that the Dobot has 4 joints by default.

[targetJointTrajPub,targetJointTrajMsg] = rospublisher('/dobot_magician/target_joint_states');
trajectoryPoint = rosmessage("trajectory_msgs/JointTrajectoryPoint");
trajectoryPoint.Positions = jointTarget;
targetJointTrajMsg.Points = trajectoryPoint;

send(targetJointTrajPub,targetJointTrajMsg);
pause(2);
%% Get current(Default) joint state of Dobot
jointStateSubscriber = rossubscriber('/dobot_magician/joint_states');   % Create a ROS Subscriber to the topic joint_states
pause(2);                                                             % Allow some time for a message to appear
DefaultJointState = jointStateSubscriber.LatestMessage.Position 
pause(2);
%% Set tool state (set suction cup)
% Turn on the tool
[toolStatePub, toolStateMsg] = rospublisher('/dobot_magician/target_tool_state');
toolStateMsg.Data = [0]; % Send 1 for on and 0 for off 
send(toolStatePub,toolStateMsg);
pause(2);

%% Get image source
i = 1;
wherepath = what('DobotCameraImgs');  %find folder path 
saveimgpath = wherepath.path;
rgbSub = rossubscriber('/camera/color/image_raw');           % subscribe to usb_cam raw image. Change whatever is in brackets to directory found through rostopic list
% rgbSub = rossubscriber('/usb_cam/image_raw');
pause(2);

figure(2);
imgIn = rgbSub.LatestMessage;
im = readImage(imgIn);
image_h = imshow(readImage(rgbSub.LatestMessage)); 

while i==1  
% % Get and Update Images   
    figure(2);
    imgIn = rgbSub.LatestMessage;
    im = readImage(imgIn);
    image_h = imshow(readImage(rgbSub.LatestMessage)); 
    image_h.CData = readImage(rgbSub.LatestMessage);
    drawnow;
    
    
    
% % Detect specific color

colour = 1; %red=1; green=2; blue=3;

R = im(:,:,1) > 55;
G = im(:,:,2) < 35;
B = im(:,:,3) < 35;
RedPixels = R & G & B;

R = im(:,:,1) < 35;
G = im(:,:,2) > 55;
B = im(:,:,3) < 35;
GreenPixels = R & G & B;

R = im(:,:,1) < 35;
G = im(:,:,2) < 35;
B = im(:,:,3) > 55;
BluePixels = R & G & B;


AreaRed = sum(RedPixels(:) == 1);
AreaGreen = sum(GreenPixels(:) == 1);
AreaBlue = sum(BluePixels(:) == 1);


if((AreaRed>AreaGreen) && (AreaRed>AreaBlue))                         %if the area of Red is greater than Green and Blue then its Red
    colour=1;
 elseif((AreaGreen>AreaRed) && (AreaGreen>AreaBlue))                    %if the area of Green is greater than Red and Blue then its Green
    colour=2;
 elseif((AreaBlue>AreaRed) && (AreaBlue>AreaGreen))                    %if the area of Blue is greater than red and Green then its Blue
    colour=3;                  
 else
    colour=0;
end  



subplot(2,3,2);
imshow(R);
title('R'); 
subplot(3,3,3);
imshow(RedPixels);
title('Red'); 
subplot(2,1,2);
imshow(im);
title('im'); 


% Get location of white area

% I1 = convertRGBtoGS(RedPixels); 
% I2 = convertGStoBW(I1,0.35); 
I2= RedPixels;
[y1,x1] = find(I2==0) ;   % get black pixels to reduce white background 
I = im(min(y1):max(y1),min(x1):max(x1),:) ;  % Get the only black part of the image 
I1 = convertRGBtoGS(I); 
I2 = convertGStoBW(I1,0.35); 
[y,x] = find(I2) ;   % get white pixels in image I 
x = x+min(x1) ; y = y+min(y1) ;   % white pxiels in original image 
imshow(im)
hold on
plot(x,y,'.r')

objectX = ((max(x) - min(x))/2)+min(x);
objectY = (max(y) - min(y))/2+min(y);

hold on
plot(objectX,objectY,'.b')

% Get the size of the input image
[rows, cols, channels] = size(im);

imgX = cols/2;
imgY = rows/2;

difX = -(imgX - objectX);
difY = imgY - objectY;

hold on
plot(imgX,imgY,'.y')

i=i+1;

end 

%% Colour detection + Coordinates + moving to coords
% PickupScenario;                   % A hardcoded scenario for picking
                                    % and dropping objects
objectcoords = itemDetectCoords()

% jointStateSubscriber = rossubscriber('/dobot_magician/joint_states');   % Create a ROS Subscriber to the topic joint_states
% pause(2);                                                             % Allow some time for a message to appear
% newJointState = jointStateSubscriber.LatestMessage.Position 
% pause(2);

reditemlocation= [objectcoords(1, 2) objectcoords(1, 1) 0.05];
itemtarget = reditemlocation;
itemtargetAbove = itemtarget;
itemtargetAbove(3) = itemtarget(3) + 0.05;
endEffectorRotation = [0,0,0];

[targetEndEffectorPub, targetEndEffectorMsg] = rospublisher('/dobot_magician/target_end_effector_pose');
targetEndEffectorMsg.Position.X = itemtargetAbove(1);
targetEndEffectorMsg.Position.Y = itemtargetAbove(2);
targetEndEffectorMsg.Position.Z = itemtargetAbove(3);

qua = eul2quat(endEffectorRotation);
targetEndEffectorMsg.Orientation.W = qua(1);
targetEndEffectorMsg.Orientation.X = qua(2);
targetEndEffectorMsg.Orientation.Y = qua(3);
targetEndEffectorMsg.Orientation.Z = qua(4);

send(targetEndEffectorPub, targetEndEffectorMsg);
pause(3);

jointStateSubscriber = rossubscriber('/dobot_magician/joint_states');   % Create a ROS Subscriber to the topic joint_states
pause(2);                                                             % Allow some time for a message to appear
newJointState = jointStateSubscriber.LatestMessage.Position 
pause(10);

% Set target joint state
jointTarget = newJointState; % Remember that the Dobot has 4 joints by default.

[targetJointTrajPub,targetJointTrajMsg] = rospublisher('/dobot_magician/target_joint_states');
trajectoryPoint = rosmessage("trajectory_msgs/JointTrajectoryPoint");
trajectoryPoint.Positions = jointTarget;
targetJointTrajMsg.Points = trajectoryPoint;

send(targetJointTrajPub,targetJointTrajMsg);
pause(2);


%% RGB to GS

function [imgGS] = convertRGBtoGS(imgRGB)

% Get the size of the input image
[rows, cols, ~] = size(imgRGB);

% Create an empty matrix for the new greyscale image
imgGS = zeros(rows,cols);

for i = 1:rows
    for j = 1:cols
        % greyscale value = 0.2989 * R + 0.5870 * G + 0.1140 * B
        imgGS(i,j) = 0.2989 * imgRGB(i,j,1) + 0.5870 * imgRGB(i,j,2) + 0.1140 * imgRGB(i,j,3);
    end
end

imgGS = uint8(imgGS);
imshow(imgGS);

end
%% GS to BW

function [imgBW] = convertGStoBW(imgGS, threshold)

% Get the size of the input image
[rows, cols, ~] = size(imgGS);

%create an empty matrix for the binary image
imgBW = zeros(rows,cols);

for i = 1:rows
    for j = 1:cols
        if imgGS(i,j) <= threshold*256
            imgBW(i,j) = 1;
        end
    end
end

imgBW = logical(imgBW);
imshow(imgBW);

end