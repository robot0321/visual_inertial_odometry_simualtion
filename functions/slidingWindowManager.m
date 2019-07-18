%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% <slidingWindowManager.m specification>
% 1. Managing sliding windows named LiveTracks & DeadTracks
% 2. In real application, world_idx in each tracks are not available
% 3. DeadTrack saves (current) untrakced tracks temporarily
% 
% INPUT    : previous LiveTracks, currStep (index), RS_valid_index, featGroup, trajParams
% OUTPUT  : current LiveTracks & DeadTracks
% FUNCTION: stack the features on the LiveTracks and move some tracks which is untracked from LiveTrack to DeadTrack
% 
% Tuning Parameter: NONE
% 
% Copyright (c) 2019 JaeYoung Chung (robot0321@github) All Rights Reserved
% Lisence: GNU GENERAL PUBLIC LICENSE Version 3, 29 June 2007
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [LiveTracks, DeadTracks] = slidingWindowManager(LiveTracks, currStep, RS_valid_index, featGroup, trajParams)
    tempLiveTracks = cell(1,trajParams.featGenParams.Nfeatures);
    misTrackCount = 0;
    for trackNumber = 1:numel(RS_valid_index)
        tempLiveTracks{featGroup.feat_intrscCurrIdx(RS_valid_index(trackNumber))} = ...
            struct('world_idx', [LiveTracks{featGroup.feat_intrscPrevIdx(RS_valid_index(trackNumber))}.world_idx, featGroup.feat_intrscCurrIdx(RS_valid_index(trackNumber))], ...
                   'frame', [LiveTracks{featGroup.feat_intrscPrevIdx(RS_valid_index(trackNumber))}.frame, currStep], ...
                   'pts', [LiveTracks{featGroup.feat_intrscPrevIdx(RS_valid_index(trackNumber))}.pts, featGroup.feat_currTrckPixel(:,RS_valid_index(trackNumber))]);
       if(featGroup.feat_intrscCurrIdx(RS_valid_index(trackNumber)) ~= featGroup.feat_intrscPrevIdx(RS_valid_index(trackNumber)))
           misTrackCount = misTrackCount + 1;
       end
    end
    fprintf('misTracking: %d \n', misTrackCount);
    
    % Stacking newTracks on the LiveTracks
    for trackNumber = 1:numel(featGroup.feat_currNewValidx)
        tempLiveTracks{featGroup.feat_currNewValidx(trackNumber)} = ...
            struct('world_idx', featGroup.feat_currNewValidx(trackNumber), ...
                  'frame', currStep, ...
                  'pts', featGroup.feat_currNewPixel(:,trackNumber));
    end
    
    % Moving ended tracks from LiveTracks to DeadTracks 
    DeadTracks = cell(1,trajParams.featGenParams.Nfeatures);
    for trackNumber = 1:numel(featGroup.feat_prevDeadValidx)
        DeadTracks{featGroup.feat_prevDeadValidx(trackNumber)} = LiveTracks{featGroup.feat_prevDeadValidx(trackNumber)};
    end
    
    LiveTracks = tempLiveTracks;
end