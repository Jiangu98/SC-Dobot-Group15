%%
clc
clf
clear all

%% Setup
rosshutdown;
name = 'Dobot';
workspace = [-1 1 -1 1 -1 1];
scale = 0.5;
rosinit;
% DobotMagician;

%% Modelling Robot
% L1 = Link('d', 0.103, 'a', 0, 'alpha', pi/2, 'qlim', deg2rad([-135 135]));
% L2 = Link('d', 0, 'a', 0.140, 'alpha', -pi, 'qlim', deg2rad([5 80]));
% L3 = Link('d', 0, 'a', 0.160, 'alpha', 0, 'qlim', deg2rad([-5 85]));
% L4 = Link('d', 0, 'a', 0, 'alpha', 0, 'qlim', [-pi/2 pi/2]);
% L5 = Link('d', 0, 'a', 0.070, 'alpha', 0, 'qlim', deg2rad([-85 85]));

L1 = Link('d',0.137,'a',0,'alpha',-pi/2,'offset',0,'qlim', deg2rad([-135 135]));
L2 = Link('d',0,'a',0.1393,'alpha',0,'offset',-pi/2, 'qlim', deg2rad([5 80]));
L3 = Link('d',0,'a',0.16193,'alpha',0,'offset',0, 'qlim', deg2rad([15 170]));
L4 = Link('d',0,'a',0.0597,'alpha',pi/2,'offset',-pi/2, 'qlim', [-pi/2 pi/2]);

Dobotsim = SerialLink([L1 L2 L3 L4], 'name', name);
q = zeros(1,4);
figure(1);
Dobotsim.plot(q,'workspace', workspace, 'scale', scale)
Dobotsim.teach;
hold on;

%% IR Dobot initialisation
% dobot = DobotMagician();
% 
% % Publish custom joint target
% joint_target = [0.0,0.4,0.3,0.0];
% dobot.PublishTargetJoint(joint_target);
% 
% % Publish custom end effector pose
% end_effector_position = [0.2,0,0.1];
% end_effector_rotation = [0,0,0];
% dobot.PublishEndEffectorPose(end_effector_position,end_effector_rotation);

%% Camera test RGB
% rostopic list
i = 0;
figure(2)
rgbSub = rossubscriber('/usb_cam/image_raw');
pause(1);
image_h = imshow(readImage(rgbSub.LatestMessage));

set(gcf,'units','normalized','outerposition',[0 0 0.2 0.2]);

tic
while i<250
 image_h.CData = readImage(rgbSub.LatestMessage);
 drawnow;
 toc;
 i = i+1;
end

%% Camera Test Compressed 
% rostopic list
i = 0;
figure(3)
depthSub = rossubscriber('/usb_cam/image_raw/compressed');
pause(1);
depthImage_h = imshow(readImage(depthSub.LatestMessage));

set(gcf,'units','normalized','outerposition',[0 0 0.2 0.2])


tic
while i<250
 depthImage_h.CData = readImage(depthSub.LatestMessage);
 drawnow;
 toc;
 i = i+1;
end

%% Point Cloud 
rgbSub = rossubscriber('/usb_cam/image_raw');
pointsSub = rossubscriber('/usb_cam/image_raw/compressed');
pause(5); 

% Get the first message and plot the non coloured data
pointMsg = pointsSub.LatestMessage;
pointMsg.PreserveStructureOnRead = true;
cloudPlot_h = scatter3(pointMsg,'Parent',gca);
drawnow();

% Loop until user breaks with ctrl+c
while 1
    % Get latest data and preserve structure in point cloud
    pointMsg = pointsSub.LatestMessage;
    pointMsg.PreserveStructureOnRead = true;             
    
    % Extract data from msg to matlab
    cloud = readXYZ(pointMsg); 
    img = readImage(rgbSub.LatestMessage);
    
    % Put in format to update the scatter3 plot quickly
    x = cloud(:,:,1);
    y = cloud(:,:,2);
    z = cloud(:,:,3);
    r = img(:,:,1);
    g = img(:,:,2);
    b = img(:,:,3);
    
    % Update the plot
    set(cloudPlot_h,'CData',[r(:),g(:),b(:)]);
    set(cloudPlot_h,'XData',x(:),'YData',y(:),'ZData',z(:));
    drawnow();
end
