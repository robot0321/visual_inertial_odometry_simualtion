%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% <SIM5 specification>
% 1. Visual system from SIM2 to VO
% 2. constant velocity propagation model
% 3. 
% 
% Copyright with Jae Young Chung, robot0321 at github 
% Lisence: GNU GENERAL PUBLIC LICENSE Version 3, 29 June 2007
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc; clear; close all;
rng(10);

%% environment setting
Nfeatures = 500; % the number of features
% x,y: 'Nfeatures' number of features, in range (-50,100)
feat_position = [rand(2,Nfeatures)*100 - 25; rand(1,Nfeatures)*30-15];

%% path setting 
fwd_speed = 2; % speed along the forward direction
rot_time = 10; % steps needs to be rotated

% body position and attitude in world view
tw_bw  = [0:fwd_speed:50, 50*ones(1,rot_time), 50*ones(size(0:fwd_speed:50)), 50*ones(1,rot_time),  50:-fwd_speed:0, 0*ones(1,rot_time), zeros(size(50:-fwd_speed:0));
         zeros(size(0:fwd_speed:50)), 0*ones(1,rot_time), 0:fwd_speed:50, 50*ones(1,rot_time), 50*ones(size(50:-fwd_speed:0)), 50*ones(1,rot_time), 50:-fwd_speed:0;
         zeros(1, size(0:fwd_speed:50,2)+size(0:fwd_speed:50,2)+size(50:-fwd_speed:0,2)+size(50:-fwd_speed:0,2)+ rot_time*3)];
heading = pi/180*[zeros(size(0:fwd_speed:50)), linspace(0,90,rot_time), 90*ones(size(0:fwd_speed:50)), ...
            linspace(90,180,rot_time), 180*ones(size(50:-fwd_speed:0)),linspace(180,270,rot_time), 270*ones(size(50:-fwd_speed:0))]; % yaw (rad)
pitch = zeros(size(heading));
roll = pi*ones(size(heading));
Rbw = angle2dcm(heading, pitch, roll); % dcm rotation matrix

% Transformation matrix world to body T^w_b
tb_bw = zeros(3,1,size(tw_bw,2));
for i=1:size(tw_bw,2)
    tb_bw(:,1,i) = -Rbw(:,:,i)*tw_bw(:,i); 
end
Tbw = [Rbw, tb_bw; zeros(1,3,size(tw_bw,2)),ones(1,1,size(tw_bw,2))]; % T^b_w

%% camera setting
Tcb = [angle2dcm(pi/2, 0,pi/2), -[0;0;1e-3]; zeros(1,3), 1]; % T^c_b, extrinsic
% -> the origin of camera frame is at the (x = 1e-3m, y = 0, z = 0) in body frame
f = 433; % focal_length
cx = 320.5; cy=240.5; % 640 x 480 (arbitariliy selected)
K = [f, 0, cx;
     0, f, cy;
     0, 0, 1]; % intrinsic matrix

mindist = 2; % minimum distance from a feature to camera principal point
maxdist = 30; % maximum distance "

%% driving robot


% 등속모델이나 회전을 가정하려면 속도가 연속이어야 해서... 경로를 만들어야한다.

