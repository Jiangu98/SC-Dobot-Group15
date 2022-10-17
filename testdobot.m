%%
clc
clf
clear all

%% Setup and Initialising stuff 
rosshutdown;
rosinit;                    % If driver is on your laptop
% rosinit(ip address);      % If connecting to UTS ip
% startup_rvc;
name = 'Dobot';
pause(1);

%% 
dobotStateSub = rossubscriber('/dobot_magician/joint_states');
receive(dobotStateSub,2)
msg = dobotStateSub.LatestMessage;

%%