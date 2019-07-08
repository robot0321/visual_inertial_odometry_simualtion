%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% <SIM6 specification>
% 0. world, body, camera frame - 3-D position, attitude applied
% 1. visual system from SIM2 + IMU system from SIM3
% 2. Features with Distance, Attitude, Intrinsic Matrix Constraint 
% 3. Visualization in 3D dimension and projected camera view
% 4. Feature Tracks on image plane
% 5. Epipoar Constraint cost with matched features
% 
% world frame (normal x,y,z), body frame (forward-x, right-y, down-z),
% camera frame(forward-z, right-x, down-y) 
% intrinsic matrix (K), extrinsic matrix (Tcb) applied
% 
% External functions: world2pixel.m
% Tuning Parameter: min/maxdist, PixelErr, DistanceThreshold(fundamental
% matrix), distCoeff(& error)
% 
% Copyright ? 2011 JaeYoung Chung (robot0321@github) All Rights Reserved
% Lisence: GNU GENERAL PUBLIC LICENSE Version 3, 29 June 2007
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc; clear; close all;
rng(10);
addpath('./functions');

%% environment setting
Nfeatures = 1000; % the number of features
% x,y: 'Nfeatures' number of features, in range (-50,100)
feat_position = [rand(2,Nfeatures)*100 - 25; rand(1,Nfeatures)*30-15];

%% path setting
fwd_speed = 4; % speed along the forward direction
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

%% camera setting
Tcb = [angle2dcm(pi/2, 0,pi/2), -[0;0;1e-3]; zeros(1,3), 1]; % T^c_b, extrinsic
% -> the origin of camera frame is at the (x = 1e-3m, y = 0, z = 0) in body frame
f = 433; % focal_length
cx = 320.5; cy=240.5; % 640 x 480 (arbitariliy selected)
px = 640; py =480;    % 640 x 480 (arbitariliy selected)
K = [f, 0, cx;
     0, f, cy;
     0, 0, 1]; % intrinsic matrix

mindist = 2; % minimum distance from a feature to camera principal point
maxdist = 40; % maximum distance "

% noise on camera plane (pixel noise) about 0.1 ~ 2
isCamPixelError = false;
PixelErr = 1; 

% Whether applying undistortion error//distCoeff: [k1, k2, p1, p2, k3, k4, k5]
% distortion error is applied at world2pixel() function
isDistorted = false;
distCoeff = [-0.5359, 0.3669, -0.0035, 0.0073, 0.1043, -0.0387, -0.0127]; % example
distortOrder = [7, 4]; % the order of distortion/undistortion(with error) model(k1~5,p1,2==7 // k1,2,p1,2==4)
errorFactor = [0.1, 0.072, 0.0007, 0.0014];
distortParams = struct('isDistorted', isDistorted, 'distCoeff', distCoeff, ...
                       'distortOrder', distortOrder, 'errorFactor',errorFactor);
                   
featureTracks = {};

%% driving robot
for i=1:size(tw_wb,2)
    % Select constrainted features
    currpos = tw_wb(:,i);
    curratt = Rbw; 

    % distance constraint
    feat_dist = sqrt(sum((feat_position - currpos).^2, 1)); 
    feat_dist_validx = find(feat_dist>mindist & feat_dist<maxdist); 
    
    % camera attitude constraint
    feat_cam = Tcb*Tbw(:,:,i)*[feat_position(1:3,feat_dist_validx); ones(1,size(feat_dist_validx,2))];
    feat_cam_validx = feat_cam(3,:)>0; 
    
    % camera intrinsic matrix constraint
    feat_pixel = K*(feat_cam(1:3,feat_cam_validx)./feat_cam(3,feat_cam_validx));
    feat_pixel_validx = feat_pixel(1,:)>0 & feat_pixel(1,:)<px ...
                        & feat_pixel(2,:)>0 & feat_pixel(2,:)<py; 
                    
    % The features with all the constriant in world frame                
    feat_world_validx = feat_dist_validx(feat_cam_validx); feat_world_validx = feat_world_validx(feat_pixel_validx);

    % index saving for optical flow
    k=i-1;
    if(i==1), feat_prevValidx = feat_world_validx; k=i; end
    feat_currValidx = feat_world_validx;
    
    % build Tracks of tracked (exist on both prev&curr) features 
    feat_currNewValidx = setdiff(feat_currValidx, feat_prevValidx);
%     if(i==1), feat_currNewValidx = feat_currValidx; end
    [feat_intrsectValidx, ~, ~] = ...
        intersect(feat_prevValidx, feat_currValidx); % feature which is tracked 

    
    % Features on Camera Plane 
    [feat_prevTrckPixel, feat_prevTrckNormal] = world2pixelNnormal(feat_position(:,feat_intrsectValidx), K, Tcb*Tbw(:,:,k), distortParams);
    feat_currNewPixel                         = world2pixelNnormal(feat_position(:,feat_currNewValidx), K, Tcb*Tbw(:,:,i), distortParams);  % features which is newly appeared 
    [feat_currTrckPixel, feat_currTrckNormal] = world2pixelNnormal(feat_position(:,feat_intrsectValidx), K, Tcb*Tbw(:,:,i), distortParams);
    % caution: not-tracked previous features are not drew (because not interested)
    
    if(isCamPixelError)
        currNewPixelErr = PixelErr*randn(size(feat_currNewPixel));
        currTrckPixelErr = PixelErr*randn(size(feat_currTrckPixel));
        feat_currNewPixel = feat_currNewPixel + currNewPixelErr;
        feat_currTrckPixel = feat_currTrckPixel + currTrckPixelErr;
    end
    
    % Fundamental Matrix with RANSAC
    [~, valid_index] = estimateFundamentalMatrix(feat_prevTrckPixel(1:2,:)', feat_currTrckPixel(1:2,:)',...
                        'Method', 'RANSAC', 'DistanceThreshold', 0.7);
    % Testing Inlier Ratio
    sum(valid_index)/size(feat_prevTrckPixel,2)
    
    % epipolar constraint (index k == b1 time / index i == b2 time)
    R_b1b2 = Tbw(1:3,1:3,k)*Tbw(1:3,1:3,i)'; % R_b1w * R_wb2
    dp_b1_b1b2 = Tbw(1:3,4,k) - R_b1b2*Tbw(1:3,4,i) + 1e-7*rand(3,1); % Pb1_b1w - Rb1b2 * Pb2_b2w
    R = Tcb(1:3,1:3) * R_b1b2 * Tcb(1:3,1:3)'; % Rc1c2 = Rc1b1 * Rb1b2 * Rb2c2
    dp = Tcb(1:3,4) + Tcb(1:3,1:3)*dp_b1_b1b2 + (-R*Tcb(1:3,4));
    d_list = [];
    for j=1:length(feat_prevTrckNormal)
        % epipolar constraint
        dpx = [0, -dp(3), dp(2); dp(3), 0, -dp(1); -dp(2), dp(1), 0];
        d_list = [d_list; feat_prevTrckNormal(:,j)'* dpx * R * feat_currTrckNormal(:,j)];
%         d_list = [d_list; dot(-cross(feat_prevTrckNormal(:,j), R*feat_currTrckNormal(:,j)), dp)];
   end
    
    figure(3);
    histogram(sort(d_list),31);
    axis([-1e-8, 1e-8, 0, 50])
    %% draw figures
    figure(1); 
    % 3-D features, robot, path and the features in view
    subplot(1,2,1); 
    scatter3(feat_position(1,:), feat_position(2,:),feat_position(3,:),'b'); hold on;
    plot3(tw_wb(1,:), tw_wb(2,:), tw_wb(3,:),'r'); 
    plot3([currpos(1)*ones(1,size(feat_world_validx,2)); feat_position(1,feat_world_validx)], ...
         [currpos(2)*ones(1,size(feat_world_validx,2)); feat_position(2,feat_world_validx)], ...
         [currpos(3)*ones(1,size(feat_world_validx,2)); feat_position(3,feat_world_validx)],'k'); 
  
    scatter3(currpos(1),currpos(2), currpos(3), 50,'g','filled');
    axis equal; hold off;
    
    subplot(1,2,2); 
    % current & previous features on camera image plane, and optical flow
    scatter(feat_currTrckPixel(1,:), feat_currTrckPixel(2,:),50,'r'); hold on;
    if ~isempty(feat_currNewPixel), scatter(feat_currNewPixel(1,:), feat_currNewPixel(2,:),50,'r'); hold on; end % avoid the error when it is empty
    scatter(feat_prevTrckPixel(1,:), feat_prevTrckPixel(2,:),'g'); 
    
    % optical flow between tracked features
    plot([feat_prevTrckPixel(1,valid_index); feat_currTrckPixel(1,valid_index)], [feat_prevTrckPixel(2,valid_index); feat_currTrckPixel(2,valid_index)],'-y');
    plot([feat_prevTrckPixel(1,~valid_index); feat_currTrckPixel(1,~valid_index)], [feat_prevTrckPixel(2,~valid_index); feat_currTrckPixel(2,~valid_index)],'-k');
    axis equal; hold off;
    axis([0, 640, 0, 480]);
    
    % Valid or Not along the distance to feature 
%     figure(2);
%     stem(sort(sqrt(sum((currpos-feat_position(:,feat_intrsectValidx(valid_index))).^2)))); hold on;
%     stem(sort(sqrt(sum((currpos-feat_position(:,feat_intrsectValidx(~valid_index))).^2))));
                    
%% For next step
    feat_prevValidx = feat_currValidx;
%     feat_prevErr = struct('NewPixErr',currNewPixelErr,'TrckPixErr',currTrckPixelErr); % noised pixels of previous step 
end