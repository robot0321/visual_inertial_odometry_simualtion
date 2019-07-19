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
% Copyright (c) 2019 JaeYoung Chung (robot0321@github) All Rights Reserved
% Lisence: GNU GENERAL PUBLIC LICENSE Version 3, 29 June 2007
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc; clear; close all;
rng(10);
addpath('./functions');
addpath('./trajectories');

dual_left_monitor = 1; 
if(dual_left_monitor) 
    figure(1); set(gcf,'Position',[-1900 50 1800 900]); % This is my workspace setting. 
else 
    figure(1); set(gcf,'Position',[100 50 1800 900]); % ... usually this option is proper OR set as you wish
end

%% environment setting
% Trajectory LIST --> select among 'traj1', 'traj2', 'traj3'
trajtype = 'traj3';
% Get the trajectory parameters from its type.   
trajParams = trajSettings(trajtype); % Set the proper options fitted with the selected trajectory

% Getting ready-made trajectories from exampleDataX.mat file
[Tbw, traj_world_wb] = getTrajectory(trajParams.fileName);

% Generating features along the trajectory
feat_position = featureGeneration(traj_world_wb, trajParams.featGenParams);

%% camera setting
% camera parameter setting, write the error model you need on second parameter
% 'pixelErr', 'distortion', 'mistrack'
cameraParams = cameraSettings(trajParams.distRange, {});

% Distance Threshold in finding the 8-point RANSAC
estFundaThreshold = 0.2;

% Feature Tracks
LiveTracks = {};
DeadTracks = {};

%% driving robot
startIdx = 1; featGroup=struct(); t=0;
for currStep=startIdx:size(Tbw,3)
    % Index saving for optical flow
    if(currStep==startIdx)
        featGroup.feat_prevValidx = []; 
        robotParamsPrev = struct('feat_position', feat_position, 'Tbw',Tbw(:,:,currStep));
    end
    robotParams = struct('feat_position', feat_position, 'Tbw',Tbw(:,:,currStep));

    %% Feature Tracking & Optical Flow
    featGroup = trackingStep(featGroup, robotParamsPrev, robotParams, cameraParams);

    % Calculating the cost of the epipolar constraint 
    d_list = epipolarConstraint(featGroup, robotParams, robotParamsPrev, cameraParams);
    
    %% data consistency check
    % consistency_check: 'RANSAC', 'priorSAC'
    consistency_check = 'priorSAC';
    switch(consistency_check)
        case 'RANSAC'
            % Fundamental Matrix with RANSAC
            if (size(featGroup.feat_prevTrckPixel,2) >= 8)
                [~, RSvalid_logic] = estimateFundamentalMatrix(featGroup.feat_prevTrckPixel(1:2,:)', featGroup.feat_currTrckPixel(1:2,:)',...
                                    'Method', 'RANSAC', 'DistanceThreshold', estFundaThreshold);
            else, RSvalid_logic = ones(1,size(featGroup.feat_currTrckPixel,2)); 
            end
        case 'priorSAC'
            RSvalid_logic = d_list < 1e-8 & d_list > -1e-8;
    end

    % Testing Inlier Ratio
    sum(RSvalid_logic)/size(featGroup.feat_prevTrckPixel,2)
    
    % update index with the results of RANSAC
    featGroup.feat_prevDeadValidx = [featGroup.feat_prevDeadValidx, featGroup.feat_intrscPrevIdx(~RSvalid_logic)];
    
    %% Managing feature tracks 
    % Stacking existed Tracks with new features on the LiveTracks
    RS_valid_index = find(RSvalid_logic);
    [LiveTracks, DeadTracks] = slidingWindowManager(LiveTracks, currStep, RS_valid_index, featGroup, trajParams);
    
    % from Tracks, Get the 3D position 
    reprodFeat = track2feat3D(DeadTracks, Tbw, 1e6, robotParams, cameraParams);  
    TrackParams = struct(); TrackParams.reprodFeat = reprodFeat;
    TrackParams.LiveTracks = LiveTracks; TrackParams.DeadTracks = DeadTracks; 
    
    %% draw figures
    drawFigures(traj_world_wb, feat_position, currStep, RSvalid_logic,TrackParams, featGroup, cameraParams, d_list)
    
    %% For next step
    featGroup.feat_prevValidx = sort([featGroup.feat_intrscCurrIdx(RS_valid_index), featGroup.feat_currNewValidx]);
   
    %% moving features
    robotParamsPrev = struct('feat_position', feat_position, 'Tbw',Tbw(:,:,currStep));
    nt = t + pi/10;
    delta_sin = 10*[sin(nt)-sin(t); -sin(nt)+sin(t); 0]; 
    feat_position(:,250:300) = feat_position(:,250:300) + delta_sin;
    t = nt;
end
