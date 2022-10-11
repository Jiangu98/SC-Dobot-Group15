%% Helpful Terminal Commands
% sudo chmod 666 /dev/ttyUSB0;
% roslaunch dobot_magician_driver dobot_magician.launch

close all;
clear all;
clc;

%% Initialise Dobot
rosshutdown;
rosinit;
[safetyStatePublisher, safetyStateMsg] = rospublisher('/dobot_magician/target_safety_status');
safetyStateMsg.Data = 2;
send(safetyStatePublisher, safetyStateMsg);
pause(24); % Long pause to wait for robot to properly initialise
fprintf('\nDobot is initialised\n');

%% Initialise Variables
% 0 = -0.07  blockZ
robotXZero = 0.184030471802;
blockZ = -0.065;
basketRimZ = 0.09;

%% Input ROSBAG

bag_filepath = "capture_1.bag";

% Check if the bag file exists
if ~exist(bag_filepath, 'file')
    message = sprintf('The file could not be found:\n%s ', bag_filepath);
    uiwait(msgbox(message));
    return;
end

bag = rosbag(bag_filepath);
selection = select(bag, 'Topic', '/device_0/sensor_1/Color_0/image/data');
message_structs = readMessages(selection);
msg = message_structs{1};

[rgb_image, ~] = readImage(msg);

%% Block Location Extraction
% Define threshold values
not_mask_top_threshold = 90;
is_mask_bottom_threshold = 160;
eliminate_blob_size = 100;
frame_realworld_width = 0.8; % in Metres
dobot_above_frame = 0.18; % in Metres, distance out of top of frame
dobot_midline = 445; % in Pixels
blockCoordinates = ColouredBlobDetection(rgb_image, not_mask_top_threshold, is_mask_bottom_threshold, eliminate_blob_size, frame_realworld_width, dobot_above_frame, dobot_midline);

locationRedBlock = [blockCoordinates(1, 2) blockCoordinates(1, 1) blockZ]
locationGreenBlock = [blockCoordinates(2, 2) blockCoordinates(2, 1) blockZ]
locationBlueBlock = [blockCoordinates(3, 2) blockCoordinates(3, 1) blockZ]
locationBasket = [0.287 0.093 basketRimZ];

%% todo Pickup Green Block and drop into Basket
%Move EE to above the green block
target = locationGreenBlock;
targetAbove = target;
targetAbove(3) = target(3) + 0.05; % Move the EE 5cm higher than the robot to prevent collisons with blocks and the basket rim
[targetEndEffectorPub, targetEndEffectorMsg] = rospublisher('/dobot_magician/target_end_effector_pose');
[targetEndEffectorPub, targetEndEffectorMsg] = MoveDobot(targetAbove)
send(targetEndEffectorPub, targetEndEffectorMsg);
pauseDuration = 3; %Seconds
pause(pauseDuration)

%Move EE down to the green block
[targetEndEffectorPub, targetEndEffectorMsg] = rospublisher('/dobot_magician/target_end_effector_pose');
[targetEndEffectorPub, targetEndEffectorMsg] = MoveDobot(target)
send(targetEndEffectorPub, targetEndEffectorMsg);

%Grip the green block
state = 1; % 1 = grip with End Effector
[toolStatePub, toolStateMsg] = rospublisher('/dobot_magician/target_tool_state');
toolStateMsg.Data = [1 state];
send(toolStatePub, toolStateMsg);
pause (pauseDuration)

%Move to the height of the basket rim
target(3) = basketRimZ;
[targetEndEffectorPub, targetEndEffectorMsg] = rospublisher('/dobot_magician/target_end_effector_pose');
[targetEndEffectorPub, targetEndEffectorMsg] = MoveDobot(target)
send(targetEndEffectorPub, targetEndEffectorMsg);
pause(pauseDuration)

%Move the EE to the basket
target = locationBasket
[targetEndEffectorPub, targetEndEffectorMsg] = rospublisher('/dobot_magician/target_end_effector_pose');
[targetEndEffectorPub, targetEndEffectorMsg] = MoveDobot(target)
send(targetEndEffectorPub, targetEndEffectorMsg);
pause(pauseDuration)

%Release the gripper
state = 0; % 0 = release End Effector Grip
[toolStatePub, toolStateMsg] = rospublisher('/dobot_magician/target_tool_state');
toolStateMsg.Data = [1 state];
send(toolStatePub, toolStateMsg);

%% todo Pickup Orange Block and drop into Basket
%Move EE to above the orange block
target = locationBlueBlock;
targetAbove = target;
targetAbove(3) = target(3) + 0.05; % Move the EE 5cm higher than the robot to prevent collisons with blocks and the basket rim
[targetEndEffectorPub, targetEndEffectorMsg] = rospublisher('/dobot_magician/target_end_effector_pose');
[targetEndEffectorPub, targetEndEffectorMsg] = MoveDobot(targetAbove)
send(targetEndEffectorPub, targetEndEffectorMsg);
pauseDuration = 3; %Seconds
pause(pauseDuration)

%Move EE down to the orange block
[targetEndEffectorPub, targetEndEffectorMsg] = rospublisher('/dobot_magician/target_end_effector_pose');
[targetEndEffectorPub, targetEndEffectorMsg] = MoveDobot(target)
send(targetEndEffectorPub, targetEndEffectorMsg);

%Grip the orange block
state = 1; % 1 = grip with End Effector
[toolStatePub, toolStateMsg] = rospublisher('/dobot_magician/target_tool_state');
toolStateMsg.Data = [1 state];
send(toolStatePub, toolStateMsg);
pause (pauseDuration)

%Move to the height of the basket rim
target(3) = basketRimZ;
[targetEndEffectorPub, targetEndEffectorMsg] = rospublisher('/dobot_magician/target_end_effector_pose');
[targetEndEffectorPub, targetEndEffectorMsg] = MoveDobot(target)
send(targetEndEffectorPub, targetEndEffectorMsg);
pause(pauseDuration)

%Move the EE to the basket
target = locationBasket
[targetEndEffectorPub, targetEndEffectorMsg] = rospublisher('/dobot_magician/target_end_effector_pose');
[targetEndEffectorPub, targetEndEffectorMsg] = MoveDobot(target)
send(targetEndEffectorPub, targetEndEffectorMsg);
pause(pauseDuration)

%Release the gripper
state = 0; % 0 = release End Effector Grip
[toolStatePub, toolStateMsg] = rospublisher('/dobot_magician/target_tool_state');
toolStateMsg.Data = [1 state];
send(toolStatePub, toolStateMsg);

%% todo Pickup Yellow Block and drop into Basket
%Move EE to above the Yellow block
target = locationRedBlock;
targetAbove = target;
targetAbove(3) = target(3) + 0.05; % Move the EE 5cm higher than the robot to prevent collisons with blocks and the basket rim
[targetEndEffectorPub, targetEndEffectorMsg] = rospublisher('/dobot_magician/target_end_effector_pose');
[targetEndEffectorPub, targetEndEffectorMsg] = MoveDobot(targetAbove)
send(targetEndEffectorPub, targetEndEffectorMsg);
pauseDuration = 3; %Seconds
pause(pauseDuration)

%Move EE down to the Yellow block
[targetEndEffectorPub, targetEndEffectorMsg] = rospublisher('/dobot_magician/target_end_effector_pose');
[targetEndEffectorPub, targetEndEffectorMsg] = MoveDobot(target)
send(targetEndEffectorPub, targetEndEffectorMsg);

%Grip the Yellow block
state = 1; % 1 = grip with End Effector
[toolStatePub, toolStateMsg] = rospublisher('/dobot_magician/target_tool_state');
toolStateMsg.Data = [1 state];
send(toolStatePub, toolStateMsg);
pause (pauseDuration)

%Move to the height of the basket rim
target(3) = basketRimZ;
[targetEndEffectorPub, targetEndEffectorMsg] = rospublisher('/dobot_magician/target_end_effector_pose');
[targetEndEffectorPub, targetEndEffectorMsg] = MoveDobot(target)
send(targetEndEffectorPub, targetEndEffectorMsg);
pause(pauseDuration)

%Move the EE to the basket
target = locationBasket
[targetEndEffectorPub, targetEndEffectorMsg] = rospublisher('/dobot_magician/target_end_effector_pose');
[targetEndEffectorPub, targetEndEffectorMsg] = MoveDobot(target)
send(targetEndEffectorPub, targetEndEffectorMsg);
pause(pauseDuration)

%Release the gripper
state = 0; % 0 = release End Effector Grip
[toolStatePub, toolStateMsg] = rospublisher('/dobot_magician/target_tool_state');
toolStateMsg.Data = [1 state];
send(toolStatePub, toolStateMsg);
