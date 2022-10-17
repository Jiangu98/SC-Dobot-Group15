% Group 15 Project - Control and Grasping for DoBot Robot
% roscore
% cd /usr/local/MATLAB/R2020a/bin 
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
% startup_rvc;
pause(1);

%% Set initialise status for Dobot
[safetyStatePublisher,safetyStateMsg] = rospublisher('/dobot_magician/target_safety_status');
safetyStateMsg.Data = 2;
send(safetyStatePublisher,safetyStateMsg);

%% Get current joint state of Dobot
jointStateSubscriber = rossubscriber('/dobot_magician/joint_states');   % Create a ROS Subscriber to the topic joint_states
pause(2);                                                               % Allow some time for a message to appear
currentJointState = jointStateSubscriber.LatestMessage.Position         % Get the latest message

%% Tool State. 1 = On. 2 = Off 
toolStateSubscriber = rossubscriber('/dobot_magician/tool_state');
pause(2); %Allow some time for MATLAB to start the subscriber
currentToolState = toolStateSubscriber.LatestMessage%.Data;

%% Set tool state (set suction cup)
% Turn on the tool
[toolStatePub, toolStateMsg] = rospublisher('/dobot_magician/target_tool_state');
toolStateMsg.Data = [0]; % Send 1 for on and 0 for off 
send(toolStatePub,toolStateMsg);

%% Set target joint state
jointTarget = [0.2,0.4,-0.3,0]; % Remember that the Dobot has 4 joints by default.

[targetJointTrajPub,targetJointTrajMsg] = rospublisher('/dobot_magician/target_joint_states');
trajectoryPoint = rosmessage("trajectory_msgs/JointTrajectoryPoint");
trajectoryPoint.Positions = jointTarget;
targetJointTrajMsg.Points = trajectoryPoint;

send(targetJointTrajPub,targetJointTrajMsg);


%% Set target end effector state
endEffectorPosition = [0.5,0.2,0.5];

[targetEndEffectorPub,targetEndEffectorMsg] = rospublisher('/dobot_magician/target_end_effector_pose');

targetEndEffectorMsg.Position.X = endEffectorPosition(1);
targetEndEffectorMsg.Position.Y = endEffectorPosition(2);
targetEndEffectorMsg.Position.Z = endEffectorPosition(3);

send(targetEndEffectorPub,targetEndEffectorMsg);









