%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% <SIM2 specification>
% 1. world, body, camera frame - 3D position, attitude are applied
% 2. Features with Distance, Attitude, Intrinsic Matrix Constraint 
% 3. Visualization in 3D dimension and projected camera view
% 4. ideal case - without any distortion, pixel error, etc
% 
% world frame (normal x,y,z), body frame (forward-x, right-y, down-z),
% camera frame(forward-z, right-x, down-y) 
% intrinsic matrix (K), extrinsic matrix (Tcb) applied
% To make it ideal, check Variable 'isCamPixelError' and 'isDistorted'
% 
% External functions: world2pixel.m
% Tuning Parameter: min/maxdist, PixelErr, DistanceThreshold(fundamental matrix),
%                   distCoeff(& error), missTrackingRatio
% 
% Copyright with Jae Young Chung, robot0321 at github 
% Lisence: GNU GENERAL PUBLIC LICENSE Version 3, 29 June 2007
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc; clear; close all;
rng(10);
addpath('./functions');

%% environment setting
Nfeatures = 700; % the number of features
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

% Whether applying undistortion error//distCoeff: 
% distortion error is applied at world2pixel() function
isDistorted = false;
distCoeff = [-0.5359, 0.3669, -0.0035, 0.0073, 0]; % example [k1, k2, p1, p2, k3]
distortOrder = [4, 4]; % the order of distortion/undistortion(with error)
errorFactor = [0.1, 0.072, 0.0007, 0.0014]; % at undistortion
distortParams = struct('isDistorted', isDistorted, 'distCoeff', distCoeff, ...
                       'distortOrder', distortOrder, 'errorFactor',errorFactor);

% Miss-tracking ratio during tracking (like KLT miss-tracking)
isMisstracked = true;
missTrackingRatio = 0.05;
                   
% Distance Threshold in finding the 8-point RANSAC
estFundaThreshold = 0.2;

% Feature Tracks
LiveTracks = cell(1,Nfeatures);
DeadTracks = cell(1,Nfeatures);

%% driving robot
for i=1:size(tw_wb,2)
    % Current Robot Position and Attitute in World Frame 
    currpos = tw_wb(:,i); % t^w_bw: 
    curratt = Rbw; 
    
    %% Feature Constraints
%     data = struct('feat_position', feat_position, 'currpos', currpos, 'Tcw', Tcb*Tbw(:,:,i), ...
%                   'mindist',mindist,'maxdist',maxdist, 'K',K, 'distortParams', distortParams, 'px',px,'py',py);
%     feat_world_validx = featureConstraint({'distance','heading','pixelRange'}, data); 
    % Distance Constraint
    feat_dist = sqrt(sum((feat_position - currpos).^2, 1)); 
    feat_dist_validx = feat_dist>mindist & feat_dist<maxdist; 
    
    % Camera Heading Constraint
    feat_cam = Tcb*Tbw(:,:,i)*[feat_position(1:3,:); ones(1,size(feat_position,2))];
    feat_cam_validx = feat_cam(3,:)>0; % camera attitude constraint
    
    % Camera Intrinsic Matrix & Pixel Range Constraint
    [feat_pixel,~] = world2pixelNnormal(feat_position, K, Tcb*Tbw(:,:,i), distortParams);
    feat_pixel_validx = feat_pixel(1,:)>0 & feat_pixel(1,:)<px ...
                        & feat_pixel(2,:)>0 & feat_pixel(2,:)<py; 
                    
    % The features with all the constriant in world frame                
    feat_world_validx = find(feat_dist_validx & feat_cam_validx & feat_pixel_validx);

    %% Feature Matching & Optical Flow
    % Index saving for optical flow
    k=i-1;
    if(i==1), feat_prevValidx = []; k=i; end
    feat_currValidx = feat_world_validx;
    
    % build Tracks of tracked (exist on both prev&curr) features 
    feat_prevDeadValidx = setdiff(feat_prevValidx, feat_currValidx);
    feat_currNewValidx = setdiff(feat_currValidx, feat_prevValidx);    % new features 
    feat_intrsectValidx = intersect(feat_currValidx, feat_prevValidx); % features which are tracked 

    % Features on Camera Plane 
    [feat_prevTrckPixel,~] = world2pixelNnormal(feat_position(:,feat_intrsectValidx), K, Tcb*Tbw(:,:,k), distortParams);
    [feat_currTrckPixel,~] = world2pixelNnormal(feat_position(:,feat_intrsectValidx), K, Tcb*Tbw(:,:,i), distortParams);
    [feat_currNewPixel,~]  = world2pixelNnormal(feat_position(:,feat_currNewValidx), K, Tcb*Tbw(:,:,i), distortParams); 
    % caution: not-tracked previous features are not drew (because not interested)
    
    % Adding Pixel Error if needed
    if(isCamPixelError)
        currNewPixelErr = PixelErr*randn(size(feat_currNewPixel));
        currTrckPixelErr = PixelErr*randn(size(feat_currTrckPixel));
        feat_currNewPixel = feat_currNewPixel + currNewPixelErr;
        feat_currTrckPixel = feat_currTrckPixel + currTrckPixelErr;
    end
    
    % Fundamental Matrix with RANSAC
    if (size(feat_prevTrckPixel,2) >= 8)
        [~, valid_index] = estimateFundamentalMatrix(feat_prevTrckPixel(1:2,:)', feat_currTrckPixel(1:2,:)',...
                            'Method', 'RANSAC', 'DistanceThreshold', estFundaThreshold);
    else, valid_index = []; 
    end
    
    % Testing Inlier Ratio
    sum(valid_index)/size(feat_prevTrckPixel,2)
    
    % Stacking Tracks with Features
    for trackNumber = 1:numel(feat_intrsectValidx)
        LiveTracks{feat_intrsectValidx(trackNumber)} = ...
            struct('world_idx', LiveTracks{feat_intrsectValidx(trackNumber)}.world_idx, ...
                   'frame', [LiveTracks{feat_intrsectValidx(trackNumber)}.frame, i], ...
                   'pts', [LiveTracks{feat_intrsectValidx(trackNumber)}.pts, feat_currTrckPixel(:,trackNumber)]);
    end
    for trackNumber = 1:numel(feat_currNewValidx)
        LiveTracks{feat_currNewValidx(trackNumber)} = ...
            struct('world_idx', feat_currNewValidx(trackNumber), 'frame', i, ...
                   'pts', feat_currNewPixel(:,trackNumber));
    end
    
    % Moving Ended Tracks
    DeadTracks = cell(1,Nfeatures);
    for trackNumber = 1:numel(feat_prevDeadValidx)
        DeadTracks{feat_prevDeadValidx(trackNumber)} = LiveTracks{feat_prevDeadValidx(trackNumber)};
        LiveTracks{feat_prevDeadValidx(trackNumber)} = [];
    end
    
    
    %% draw figures
    figure(1); 
    % 3-D features, robot, path and the features in view
    subplot(1,2,1); 
    scatter3(feat_position(1,:), feat_position(2,:),feat_position(3,:),'b'); hold on; %features
    plot3(tw_wb(1,:), tw_wb(2,:), tw_wb(3,:),'r'); % path
    plot3([currpos(1)*ones(1,size(feat_world_validx,2)); feat_position(1,feat_world_validx)], ...
         [currpos(2)*ones(1,size(feat_world_validx,2)); feat_position(2,feat_world_validx)], ...
         [currpos(3)*ones(1,size(feat_world_validx,2)); feat_position(3,feat_world_validx)],'k'); % view ray
    scatter3(currpos(1),currpos(2), currpos(3), 50,'g','filled'); % robot
    axis equal; hold off;
    
    subplot(1,2,2); 
    % current & previous features on camera image plane
    if ~isempty(feat_intrsectValidx) % avoid the error when it is empty
        scatter(feat_currTrckPixel(1,:), feat_currTrckPixel(2,:),50,'r'); hold on;
        scatter(feat_prevTrckPixel(1,:), feat_prevTrckPixel(2,:),'g');
        % draw the valid(yellow) and invalid(black) optical flows
        plot([feat_prevTrckPixel(1,valid_index); feat_currTrckPixel(1,valid_index)], [feat_prevTrckPixel(2,valid_index); feat_currTrckPixel(2,valid_index)],'-y');
        plot([feat_prevTrckPixel(1,~valid_index); feat_currTrckPixel(1,~valid_index)], [feat_prevTrckPixel(2,~valid_index); feat_currTrckPixel(2,~valid_index)],'-k');
    end
    if ~isempty(feat_currNewValidx), scatter(feat_currNewPixel(1,:), feat_currNewPixel(2,:),50,'r'); end 
    
    % optical flow between tracked features
    axis equal; hold off;
    axis([0, 640, 0, 480]);
    
    figure(2);
    % LiveTracks
    subplot(1,2,1);
    for j = 1:length(LiveTracks)
        if(~isempty(LiveTracks{j})), plot(LiveTracks{j}.pts(1,:), LiveTracks{j}.pts(2,:),'-og'); hold on; end
    end
    title('Live Tracks');
    hold off;
    
    % DeadTracks
    subplot(1,2,2);
    for j = 1:length(DeadTracks)
        if(~isempty(DeadTracks{j})), plot(DeadTracks{j}.pts(1,:), DeadTracks{j}.pts(2,:), '-or'); hold on; end
    end
    title('Dead Tracks');
    hold off;
    
%% For next step
    feat_prevValidx = feat_currValidx;
    
end
