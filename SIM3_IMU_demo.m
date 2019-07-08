%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% <SIM3 specification>
% 1. Path generator with checkpoints
% 2. Inverse INS (From world path & att To inertial data with gravity)
% 3. INS (From inertial data To world path & att) with IMU error model
% 
% Copyright ? 2011 JaeYoung Chung (robot0321@github) All Rights Reserved
% Lisence: GNU GENERAL PUBLIC LICENSE Version 3, 29 June 2007
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc; clear; close all;
rng(10);
addpath('./functions');

%% environment setting
Nfeatures = 500; % the number of features
% x,y: 'Nfeatures' number of features, in range (-50,100)
feat_position = [rand(2,Nfeatures)*100 - 25; rand(1,Nfeatures)*30-15];

%% path setting 
fwd_speed = 2; % speed along the forward direction
rot_time = 10; % steps needs to be rotated

% body position and attitude in world view
tw_wb  = [0:fwd_speed:50, 50*ones(1,rot_time), 50*ones(size(0:fwd_speed:50)), 50*ones(1,rot_time),  50:-fwd_speed:0, 0*ones(1,rot_time), zeros(size(50:-fwd_speed:0));
         zeros(size(0:fwd_speed:50)), 0*ones(1,rot_time), 0:fwd_speed:50, 50*ones(1,rot_time), 50*ones(size(50:-fwd_speed:0)), 50*ones(1,rot_time), 50:-fwd_speed:0;
         zeros(1, size(0:fwd_speed:50,2)+size(0:fwd_speed:50,2)+size(50:-fwd_speed:0,2)+size(50:-fwd_speed:0,2)+ rot_time*3)];
heading = pi/180*[zeros(size(0:fwd_speed:50)), linspace(0,90,rot_time), 90*ones(size(0:fwd_speed:50)), ...
            linspace(90,180,rot_time), 180*ones(size(50:-fwd_speed:0)),linspace(180,270,rot_time), 270*ones(size(50:-fwd_speed:0))]; % yaw (rad)
pitch = zeros(size(heading));
roll = pi*ones(size(heading));
Rbw = angle2dcm(heading, pitch, roll); % dcm rotation matrix

% Transformation matrix world to body T^w_b
tb_bw = zeros(3,1,size(tw_wb,2));
for i=1:size(tw_wb,2)
    tb_bw(:,1,i) = -Rbw(:,:,i)*tw_wb(:,i); 
end
Tbw = [Rbw, tb_bw; zeros(1,3,size(tw_wb,2)),ones(1,1,size(tw_wb,2))]; % T^b_w

%% inverse INS (path2imu) in Local frame




%% INS (imu2path)
% Make IMU data from the checkpoints to the IMU error model
% error model: 1. bias: (1)random constant bias, (2)random 1st-Markov bias
%              2. scale factor : (1)identity (2)random constant diagonal 
%                 (3)random constant with skew (4) (slow)1st-Markov skew
%                 (5)nonlinear
%              3. hysteresis 
%              4. frequency variance with time
%              
% function out = IMU(checkpoints, imu_errModel)
%     path = path_generator(checkpoints);
%     true_6motion = inverseINS(path); % (acc3, gyro3)
%     real_6motion = IMUmodel(true_6motion, imu_errModel);
%     out = real_6motion;
% end
% 


%% driving robot

