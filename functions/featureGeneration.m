%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% <featureGeneration.m specification>
% 1. Generate random feature near the trajectory
% 2. Select Approximately Uniformly distributed features, and the exact number of features 
%    
% INPUT   : Trajectory described in world frame and feature generation parameters 
% OUTPUT  : Feature position in world frame
% FUNCTION: Making random feature position near the given trajectory
% 
% Tuning Parameter: NONE
% 
% Copyright (c) 2019 JaeYoung Chung (robot0321@github) All Rights Reserved
% Lisence: GNU GENERAL PUBLIC LICENSE Version 3, 29 June 2007
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function feat_position = featureGeneration(traj_world_wb, featGenParams)
    Nfeatures = featGenParams.Nfeatures;
    step = featGenParams.step;
    xyRange = featGenParams.xyRange;
    zRange = featGenParams.zRange;
    
    pathL = floor(length(traj_world_wb)/step);
    NfeatPerL = ceil(Nfeatures/pathL);
    feat_position = [];
    for i = 1:pathL
        feat_position = [feat_position, traj_world_wb(:,step*(i-1)+1) + [(rand(2,NfeatPerL)-0.5)*xyRange; (rand(1,NfeatPerL)-0.5)*zRange]];
    end
    selectIdx = randperm(length(feat_position), Nfeatures);
    feat_position = feat_position(:,selectIdx);
end