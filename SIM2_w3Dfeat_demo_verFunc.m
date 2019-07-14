%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% <SIM2 specification>
% *** If you use single monitor, set 'dual_left_monitor' to 0 ***
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
% Copyright (c) 2019 JaeYoung Chung (robot0321@github) All Rights Reserved
% Lisence: GNU GENERAL PUBLIC LICENSE Version 3, 29 June 2007
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc; clear; close all;
rng(10);
addpath('./functions');
addpath('./trajectories');

dual_left_monitor = 1; 
if(dual_left_monitor) 
    figure(1); set(gcf,'Position',[-1700 50 1300 900]); % This is my workspace setting. 
else 
    figure(1); set(gcf,'Position',[100 50 1300 900]); % ... usually this option is proper OR set as you wish
end

%% environment setting
% Trajectory LIST --> select among 'traj1', 'traj2', 'traj3'
trajtype = 'traj1';
% Get the trajectory parameters from its type.   
trajParams = trajSettings(trajtype); % Set the proper options fitted with the selected trajectory

% Getting ready-made trajectories from exampleDataX.mat file
[Tbw, traj_world_wb] = getTrajectory(trajParams.fileName);

% Generating features along the trajectory
feat_position = featureGeneration(traj_world_wb, trajParams.featGenParams);

% % draw the features
% figure(10); scatter3(feat_position(1,:),feat_position(2,:),feat_position(3,:),'b'); hold on;
% % draw the trajectory
% plot3(traj_world_wb(1,:),traj_world_wb(2,:),traj_world_wb(3,:),'LineWidth',2,'Color','red'); hold on;
% scatter3(traj_world_wb(1,1),traj_world_wb(2,1),traj_world_wb(3,1),50,'g','filled'); 
% scatter3(traj_world_wb(1,end),traj_world_wb(2,end),traj_world_wb(3,end),50,'y','filled');
% legend( 'feats', 'path', 'start', 'stop'); axis equal; grid on; hold off;

%% camera setting
% camera parameter setting, 
cameraParams = cameraSettings(trajParams.distRange);

% Distance Threshold in finding the 8-point RANSAC
estFundaThreshold = 0.2;

% Feature Tracks
LiveTracks = {};
DeadTracks = {};

%% driving robot
startIdx = 1;
for currStep=startIdx:size(Tbw,3)
    %% Feature Constraints in Current Step
    robotParams = struct('feat_position', feat_position, 'Tbw',Tbw(:,:,currStep));
    feat_world_validx = featureConstraint({'distance','heading','pixelRange'}, robotParams, cameraParams);

    %% Feature Tracking & Optical Flow
    % Index saving for optical flow
    prevStep=currStep-1;
    if(currStep==startIdx), feat_prevValidx = []; prevStep=currStep; end
    feat_currValidx = feat_world_validx;
    
    % build Tracks of tracked (exist on both prev&curr) features 
    feat_prevDeadValidx = setdiff(feat_prevValidx, feat_currValidx);
    feat_currNewValidx = setdiff(feat_currValidx, feat_prevValidx);    % new features 
    feat_intrsectValidx = intersect(feat_currValidx, feat_prevValidx); % features which are tracked 
    feat_intrscPrevIdx = feat_intrsectValidx;
    feat_intrscCurrIdx = feat_intrsectValidx;
    
    % mistracked (mismatched ?) feature 
    % for the continuous tracking, mistracking is done on the existed feature, not ramdomly made feature.
    if(cameraParams.errorParams.misTrackParams.isMistracked && ~isempty(feat_intrsectValidx))
        numMistrackedFeat = round(length(feat_intrsectValidx) * cameraParams.errorParams.misTrackParams.mistrackingRatio);
        mistrackIdx = randperm(length(feat_intrsectValidx), numMistrackedFeat);
        newCandidate = [feat_intrsectValidx(mistrackIdx), feat_currNewValidx];
        newCandidIdx = randperm(length(newCandidate), numMistrackedFeat); % select from misTrack + newCurr
        feat_intrscCurrIdx(mistrackIdx) = newCandidate(newCandidIdx);
        [~, currMistrackIdx,~] = intersect(feat_currNewValidx, newCandidate(newCandidIdx));
        if(~isempty(currMistrackIdx))
            feat_currNewValidx(currMistrackIdx) = [];
        end
    end
    
    % Features on Camera Plane 
    robotParamsPrev = struct('feat_position', feat_position, 'Tbw',Tbw(:,:,prevStep));
    [feat_prevTrckPixel,~] = world2pixelNnormal_verFunc(feat_intrscPrevIdx, robotParamsPrev, cameraParams); % intersectValidx = feat_prevValidx(:, feat_intrscPrevIdx)
    [feat_currTrckPixel,~] = world2pixelNnormal_verFunc(feat_intrscCurrIdx, robotParams, cameraParams);      % intersectValidx = feat_currValidx(:, feat_intrscCurrIdx)
    [feat_currNewPixel,~]  = world2pixelNnormal_verFunc(feat_currNewValidx,  robotParams, cameraParams); 
    % caution: not-tracked previous features are not drew (because not interested)
    
    % Adding Pixel Error if needed
    if(cameraParams.errorParams.pixelErrParams.isCamPixelError)
        currNewPixelErr = cameraParams.errorParams.pixelErrParams.PixelErr * randn(size(feat_currNewPixel));
        currTrckPixelErr = cameraParams.errorParams.pixelErrParams.PixelErr * randn(size(feat_currTrckPixel));
        feat_currNewPixel = feat_currNewPixel + currNewPixelErr;
        feat_currTrckPixel = feat_currTrckPixel + currTrckPixelErr;
    end
    
    % Fundamental Matrix with RANSAC
    if (size(feat_prevTrckPixel,2) >= 8)
        [~, RSvalid_logic] = estimateFundamentalMatrix(feat_prevTrckPixel(1:2,:)', feat_currTrckPixel(1:2,:)',...
                            'Method', 'RANSAC', 'DistanceThreshold', estFundaThreshold);
    else, RSvalid_logic = ones(1,size(feat_currTrckPixel,2)); 
    end

    % Testing Inlier Ratio
    sum(RSvalid_logic)/size(feat_prevTrckPixel,2)
    
    % update index with the results of RANSAC
    feat_prevDeadValidx = [feat_prevDeadValidx, feat_intrscPrevIdx(~RSvalid_logic)];
    
    % Stacking existed Tracks with new features on the LiveTracks
    RS_valid_index = find(RSvalid_logic);
    tempLiveTracks = {};
    for trackNumber = 1:numel(RS_valid_index)
        tempLiveTracks{feat_intrscCurrIdx(RS_valid_index(trackNumber))} = ...
            struct('world_idx', [LiveTracks{feat_intrscPrevIdx(RS_valid_index(trackNumber))}.world_idx, feat_intrscCurrIdx(RS_valid_index(trackNumber))], ...
                   'frame', [LiveTracks{feat_intrscPrevIdx(RS_valid_index(trackNumber))}.frame, currStep], ...
                   'pts', [LiveTracks{feat_intrscPrevIdx(RS_valid_index(trackNumber))}.pts, feat_currTrckPixel(:,RS_valid_index(trackNumber))]);
    end
    
    % Stacking newTracks on the LiveTracks
    for trackNumber = 1:numel(feat_currNewValidx)
        tempLiveTracks{feat_currNewValidx(trackNumber)} = ...
            struct('world_idx', feat_currNewValidx(trackNumber), ...
                  'frame', currStep, ...
                  'pts', feat_currNewPixel(:,trackNumber));
    end
    
    % Moving ended tracks from LiveTracks to DeadTracks 
    DeadTracks = cell(1,trajParams.featGenParams.Nfeatures);
    for trackNumber = 1:numel(feat_prevDeadValidx)
        DeadTracks{feat_prevDeadValidx(trackNumber)} = LiveTracks{feat_prevDeadValidx(trackNumber)};
    end
    
    LiveTracks = tempLiveTracks;
    %% draw figures
    figure(1); 
    % 3-D features, robot, path and the features in view
    subplot(2,2,1); 
    scatter3(feat_position(1,:), feat_position(2,:),feat_position(3,:),'b'); hold on; %features
    plot3(traj_world_wb(1,:), traj_world_wb(2,:), traj_world_wb(3,:),'r'); % path
    plot3([traj_world_wb(1,currStep)*ones(1,size(feat_world_validx,2)); feat_position(1,feat_world_validx)], ...
         [traj_world_wb(2,currStep)*ones(1,size(feat_world_validx,2)); feat_position(2,feat_world_validx)], ...
         [traj_world_wb(3,currStep)*ones(1,size(feat_world_validx,2)); feat_position(3,feat_world_validx)],'k'); % view ray
    scatter3(traj_world_wb(1,currStep),traj_world_wb(2,currStep),traj_world_wb(3,currStep), 50,'g','filled'); % robot position
%     yaw = dcm2angle(Tbw(1:3,1:3,currStep));
%     quiver3(traj_world_wb(1,currStep),traj_world_wb(2,currStep),traj_world_wb(3,currStep), ...
%              3*cos(yaw*pi/180),3*sin(yaw*pi/180),yaw*0); % robot heading
    axis equal; grid on; hold off; xlabel('x'); ylabel('y');
    
    subplot(2,2,2); 
    % current & previous features on camera image plane
    if ~isempty(feat_intrsectValidx) % avoid the error when it is empty
        scatter(feat_currTrckPixel(1,:), feat_currTrckPixel(2,:),50,'r'); hold on;
        scatter(feat_prevTrckPixel(1,:), feat_prevTrckPixel(2,:),'g');
        % draw the valid(yellow) and invalid(black) optical flows
        plot([feat_prevTrckPixel(1,RSvalid_logic); feat_currTrckPixel(1,RSvalid_logic)], [feat_prevTrckPixel(2,RSvalid_logic); feat_currTrckPixel(2,RSvalid_logic)],'-y');
        plot([feat_prevTrckPixel(1,~RSvalid_logic); feat_currTrckPixel(1,~RSvalid_logic)], [feat_prevTrckPixel(2,~RSvalid_logic); feat_currTrckPixel(2,~RSvalid_logic)],'-k');
    end
    if ~isempty(feat_currNewValidx), scatter(feat_currNewPixel(1,:), feat_currNewPixel(2,:),50,'r'); end 
    % optical flow between tracked features
    hold off; xlabel('x'); ylabel('y');
    axis([0, cameraParams.px, 0, cameraParams.py]);
    set(gca,'XAxisLocation','top','YAxisLocation','left','ydir','reverse');
    
    % LiveTracks
    subplot(2,2,3);
    for j = 1:length(LiveTracks)
        if(~isempty(LiveTracks{j})), plot(LiveTracks{j}.pts(1,:), LiveTracks{j}.pts(2,:),'-og'); hold on; end
    end
    title('Live Tracks'); xlabel('x'); ylabel('y');
    hold off; axis([0, cameraParams.px, 0, cameraParams.py]);
    set(gca,'XAxisLocation','top','YAxisLocation','left','ydir','reverse');
    
    % DeadTracks
    subplot(2,2,4);
    for j = 1:length(DeadTracks)
        if(~isempty(DeadTracks{j})), plot(DeadTracks{j}.pts(1,:), DeadTracks{j}.pts(2,:), '-or'); hold on; end
    end
    title('Dead Tracks');
    hold off; axis([0, cameraParams.px, 0, cameraParams.py]);
    set(gca,'XAxisLocation','top','YAxisLocation','left','ydir','reverse');
    
    drawnow();
%% For next step
    feat_prevValidx = sort([feat_intrscCurrIdx(RS_valid_index), feat_currNewValidx]);
    
end
