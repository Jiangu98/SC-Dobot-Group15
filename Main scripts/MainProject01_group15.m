% Group 15 Project - Control and Grasping for DoBot Robot
% roscore
% roslaunch dobot_magician_driver dobot_magician.launch
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

%% Setup and Initialising stuff 
rosshutdown;
rosinit;                    % If driver is on your laptop
% rosinit(ip address);      % If connecting to UTS ip
startup_rvc;
name = 'Dobot';
pause(1);

%% Check Safety Status of Robot
safetyStatusSubscriber = rossubscriber('/dobot_magician/safety_status');
pause(2); %Allow some time for MATLAB to start the subscriber
currentSafetyStatus = safetyStatusSubscriber.LatestMessage.Data;

%% Set initialise status for Dobot
[safetyStatePublisher,safetyStateMsg] = rospublisher('/dobot_magician/target_safety_status');
safetyStateMsg.Data = 2;
send(safetyStatePublisher,safetyStateMsg);

%% Get current joint state of Dobot
jointStateSubscriber = rossubscriber('/dobot_magician/joint_states');   % Create a ROS Subscriber to the topic joint_states
pause(2);                                                               % Allow some time for a message to appear
currentJointState = jointStateSubscriber.LatestMessage.Position         % Get the latest message

%% Get end-effector pose of Dobot. This is end effector pose without attachment
endEffectorPoseSubscriber = rossubscriber('/dobot_magician/end_effector_poses');    % Create a ROS Subscriber to the topic end_effector_poses
pause(2);                                                                           %Allow some time for MATLAB to start the subscriber
currentEndEffectorPoseMsg = endEffectorPoseSubscriber.LatestMessage;
% Extract the position of the end effector from the received message
currentEndEffectorPosition = [currentEndEffectorPoseMsg.Pose.Position.X,
                              currentEndEffectorPoseMsg.Pose.Position.Y,
                              currentEndEffectorPoseMsg.Pose.Position.Z];
% Extract the orientation of the end effector
currentEndEffectorQuat = [currentEndEffectorPoseMsg.Pose.Orientation.W,
                          currentEndEffectorPoseMsg.Pose.Orientation.X,
                          currentEndEffectorPoseMsg.Pose.Orientation.Y,
                          currentEndEffectorPoseMsg.Pose.Orientation.Z];
% Convert from quaternion to euler
[roll,pitch,yaw] = quat2eul(currentEndEffectorQuat);

%% Tool State. 1 = On. 2 = Off 
toolStateSubscriber = rossubscriber('/dobot_magician/tool_state');
pause(2); %Allow some time for MATLAB to start the subscriber
currentToolState = toolStateSubscriber.LatestMessage.Data;

%% Set tool state (set suction cup)
% Turn on the tool
[toolStatePub, toolStateMsg] = rospublisher('/dobot_magician/target_tool_state');
toolStateMsg.Data = [1]; % Send 1 for on and 0 for off 
send(toolStatePub,toolStateMsg);

%% Set target joint state
jointTarget = [0,0.4,0.3,0]; % Remember that the Dobot has 4 joints by default.

[targetJointTrajPub,targetJointTrajMsg] = rospublisher('/dobot_magician/target_joint_states');
trajectoryPoint = rosmessage("trajectory_msgs/JointTrajectoryPoint");
trajectoryPoint.Positions = jointTarget;
targetJointTrajMsg.Points = trajectoryPoint;

send(targetJointTrajPub,targetJointTrajMsg);

%% Set target end effector state
endEffectorPosition = [0.2,0,0.1];
endEffectorRotation = [0,0,0];

[targetEndEffectorPub,targetEndEffectorMsg] = rospublisher('/dobot_magician/target_end_effector_pose');

targetEndEffectorMsg.Position.X = endEffectorPosition(1);
targetEndEffectorMsg.Position.Y = endEffectorPosition(2);
targetEndEffectorMsg.Position.Z = endEffectorPosition(3);

qua = eul2quat(endEffectorRotation);
targetEndEffectorMsg.Orientation.W = qua(1);
targetEndEffectorMsg.Orientation.X = qua(2);
targetEndEffectorMsg.Orientation.Y = qua(3);
targetEndEffectorMsg.Orientation.Z = qua(4);

send(targetEndEffectorPub,targetEndEffectorMsg);


%% Take Photos from Camera. This uses usb_cam. change the subscriber line for Lab Camera
% rostopic list
i = 0;
wherepath = what('DobotCameraImgs');  %find folder path 
saveimgpath = wherepath.path;
rgbSub = rossubscriber('/usb_cam/image_raw');           % subscribe to usb_cam raw image. Change whatever is in brackets to directory found through rostopic list
pause(2);

for i=0:5;
    
    figure(2);
    imgIn = rgbSub.LatestMessage;
    raw_img = readImage(imgIn);
    file_name = sprintf('test%d.png', i);
    fullFileName = fullfile(saveimgpath, file_name);
    imgName = [saveimgpath,'/test_',num2str(i),'.png'];
    grayImg = rgb2gray(raw_img);                % convert rgb to gray
    imwrite(grayImg,imgName);                   % save image to folder
    imshow(grayImg)                             % show image 
    fprintf("test image %d saved \n",i)
    pause(1);
end

%% rosbag camera images. use this or section above for image capture

% 1. bagpath = what('Bags') <-- Use this in cmd window to find directory for next step
% 2. cd liam/405C-0F7C/SnC/SC-Dobot-Group15/Main\ scripts/Bags/ <--replace 
% 3. rostopic list -v 
% 4. rosbag record -O Projectrosbag1.bag usb_cam/image_raw --duration=5

wherepath = what('DobotCameraImgs');  %find folder path 
saveimgpath = wherepath.path;
i=0;

bagname = 'Projectrosbag1.bag';
bag = rosbag(bagname)
selection = select(bag, 'Topic', 'usb_cam/image_raw')  %change this to camera when in labs
message_structs = readMessages(selection, 30);
msg = message_structs{1};
rgb_image = readImage(msg);
figure(1);
imshow(rgb_image)
pause(1);
figure(2);
bagGrayimg = rgb2gray(rgb_image);
imshow(bagGrayimg) 
for i = 0
    file_name = sprintf('bagimg%d.png', i);
    fullFileName = fullfile(saveimgpath, file_name);
    imgName = [saveimgpath,'/bagimg_',num2str(i),'.png'];
    imwrite(bagGrayimg,imgName);
end



