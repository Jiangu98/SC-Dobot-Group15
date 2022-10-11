%%
clc
clf
clear all
%% Setup
name = 'Dobot';
workspace = [-1 1 -1 1 -1 1];
scale = 0.5;

%% Modelling Robot
L1 = Link('d', 0.103, 'a', 0, 'alpha', pi/2, 'qlim', deg2rad([-135 135]));
L2 = Link('d', 0, 'a', 0.140, 'alpha', -pi, 'qlim', deg2rad([5 80]));
L3 = Link('d', 0, 'a', 0.160, 'alpha', 0, 'qlim', deg2rad([-5 85]));
% L4 = Link('d', 0, 'a', 0, 'alpha', 0, 'qlim', [-pi/2 pi/2]);
L5 = Link('d', 0, 'a', 0.070, 'alpha', 0, 'qlim', deg2rad([-85 85]));

Dobot = SerialLink([L1 L2 L3 L5], 'name', name);
q = zeros(1,4);
Dobot.plot(q,'workspace', workspace, 'scale', scale)
Dobot.teach;