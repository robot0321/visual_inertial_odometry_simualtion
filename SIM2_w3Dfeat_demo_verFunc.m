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
%                   distCoeff(& error), misTrackingRatio
% 
% Copyright ? 2011 JaeYoung Chung (robot0321@github) All Rights Reserved
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

[Tbw, path_world_wb] = pathGeneration(fwd_speed, rot_time);

%% camera setting
cameraParams = cameraSettings();

% Distance Threshold in finding the 8-point RANSAC
estFundaThreshold = 0.2;

% Feature Tracks
LiveTracks = cell(1,Nfeatures);
DeadTracks = cell(1,Nfeatures);

%% driving robot
for currStep=1:size(Tbw,3)
    %% Feature Constraints in Current Step
    robotParams = struct('feat_position', feat_position, 'Tbw',Tbw(:,:,currStep));
    feat_world_validx = featureConstraint({'distance','heading','pixelRange'}, robotParams, cameraParams);

    %% Feature Tracking & Optical Flow
    % Index saving for optical flow
    prevStep=currStep-1;
    if(currStep==1), feat_prevValidx = []; prevStep=currStep; end
    feat_currValidx = feat_world_validx;
    
    % build Tracks of tracked (exist on both prev&curr) features 
    feat_prevDeadValidx = setdiff(feat_prevValidx, feat_currValidx);
    feat_currNewValidx = setdiff(feat_currValidx, feat_prevValidx);    % new features 
    feat_intrsectValidx = intersect(feat_currValidx, feat_prevValidx); % features which are tracked 

    % Features on Camera Plane 
    robotParamsPrev = struct('feat_position', feat_position, 'Tbw',Tbw(:,:,prevStep));
    [feat_prevTrckPixel,~] = world2pixelNnormal_verFunc(feat_intrsectValidx, robotParamsPrev, cameraParams);
    [feat_currTrckPixel,~] = world2pixelNnormal_verFunc(feat_intrsectValidx, robotParams, cameraParams);
    [feat_currNewPixel,~]  = world2pixelNnormal_verFunc(feat_currNewValidx,  robotParams, cameraParams); 
    % caution: not-tracked previous features are not drew (because not interested)
    
    % Adding Pixel Error if needed
    if(cameraParams.errorParams.pixelErrParams.isCamPixelError)
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
                   'frame', [LiveTracks{feat_intrsectValidx(trackNumber)}.frame, currStep], ...
                   'pts', [LiveTracks{feat_intrsectValidx(trackNumber)}.pts, feat_currTrckPixel(:,trackNumber)]);
    end
    for trackNumber = 1:numel(feat_currNewValidx)
        LiveTracks{feat_currNewValidx(trackNumber)} = ...
            struct('world_idx', feat_currNewValidx(trackNumber), 'frame', currStep, ...
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
    plot3(path_world_wb(1,:), path_world_wb(2,:), path_world_wb(3,:),'r'); % path
    plot3([path_world_wb(1,currStep)*ones(1,size(feat_world_validx,2)); feat_position(1,feat_world_validx)], ...
         [path_world_wb(2,currStep)*ones(1,size(feat_world_validx,2)); feat_position(2,feat_world_validx)], ...
         [path_world_wb(3,currStep)*ones(1,size(feat_world_validx,2)); feat_position(3,feat_world_validx)],'k'); % view ray
    scatter3(path_world_wb(1,currStep),path_world_wb(2,currStep),path_world_wb(3,currStep), 50,'g','filled'); % robot
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
