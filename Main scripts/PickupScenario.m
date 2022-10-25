%% Move End Effector from current to red Item
% Set target joint state
jointTarget = [-0.5609,0.6523,0.9380,-0.7854]; % Remember that the Dobot has 4 joints by default.

[targetJointTrajPub,targetJointTrajMsg] = rospublisher('/dobot_magician/target_joint_states');
trajectoryPoint = rosmessage("trajectory_msgs/JointTrajectoryPoint");
trajectoryPoint.Positions = jointTarget;
targetJointTrajMsg.Points = trajectoryPoint;

send(targetJointTrajPub,targetJointTrajMsg);
pause(3);
% Drop Down 

% Turn on the tool
[toolStatePub, toolStateMsg] = rospublisher('/dobot_magician/target_tool_state');
toolStateMsg.Data = [1]; % Send 1 for on and 0 for off 
send(toolStatePub,toolStateMsg);
pause(2);


% Move up
jointTarget = DefaultJointState; % Remember that the Dobot has 4 joints by default.

[targetJointTrajPub,targetJointTrajMsg] = rospublisher('/dobot_magician/target_joint_states');
trajectoryPoint = rosmessage("trajectory_msgs/JointTrajectoryPoint");
trajectoryPoint.Positions = jointTarget;
targetJointTrajMsg.Points = trajectoryPoint;

send(targetJointTrajPub,targetJointTrajMsg);
pause(3);
%% Move Green item to right side
% Move to x-y location 
jointTarget = [0.5179 0.5871 0.0849 -0.7854];

[targetJointTrajPub,targetJointTrajMsg] = rospublisher('/dobot_magician/target_joint_states');
trajectoryPoint = rosmessage("trajectory_msgs/JointTrajectoryPoint");
trajectoryPoint.Positions = jointTarget;
targetJointTrajMsg.Points = trajectoryPoint;

send(targetJointTrajPub,targetJointTrajMsg);
pause(3);

% Move down z a little
jointTarget = [0.5507 0.7202 0.3343 -0.7854];

[targetJointTrajPub,targetJointTrajMsg] = rospublisher('/dobot_magician/target_joint_states');
trajectoryPoint = rosmessage("trajectory_msgs/JointTrajectoryPoint");
trajectoryPoint.Positions = jointTarget;
targetJointTrajMsg.Points = trajectoryPoint;

send(targetJointTrajPub,targetJointTrajMsg);
pause(3);

% Turn off the tool
[toolStatePub, toolStateMsg] = rospublisher('/dobot_magician/target_tool_state');
toolStateMsg.Data = [0]; % Send 1 for on and 0 for off 
send(toolStatePub,toolStateMsg);
pause(1);

% Move to default joint state
jointTarget = DefaultJointState; % Remember that the Dobot has 4 joints by default.

[targetJointTrajPub,targetJointTrajMsg] = rospublisher('/dobot_magician/target_joint_states');
trajectoryPoint = rosmessage("trajectory_msgs/JointTrajectoryPoint");
trajectoryPoint.Positions = jointTarget;
targetJointTrajMsg.Points = trajectoryPoint;

send(targetJointTrajPub,targetJointTrajMsg);
pause(3);

%% Move end Effector from current joint state to Red Item
% % Move to x-y location 
% endEffectorPosition = DefaultJointState;
% 
% [targetEndEffectorPub,targetEndEffectorMsg] = rospublisher('/dobot_magician/target_end_effector_pose');
% 
% targetEndEffectorMsg.Position.X = endEffectorPosition(1);
% targetEndEffectorMsg.Position.Y = endEffectorPosition(2);
% targetEndEffectorMsg.Position.Z = endEffectorPosition(3);
% 
% send(targetEndEffectorPub,targetEndEffectorMsg);
% % Drop Down 
% 
% 
% % Turn on the tool
% [toolStatePub, toolStateMsg] = rospublisher('/dobot_magician/target_tool_state');
% toolStateMsg.Data = [0]; % Send 1 for on and 0 for off 
% send(toolStatePub,toolStateMsg);
% 
% % Move up 
% 
%% Move Red item to left side
% % Move to x-y location 
% 
% % Move down z a little 
% 
% % Turn off the tool
% [toolStatePub, toolStateMsg] = rospublisher('/dobot_magician/target_tool_state');
% toolStateMsg.Data = [0]; % Send 1 for on and 0 for off 
% send(toolStatePub,toolStateMsg);
% 
% % Move to default joint state
% currentJointState
