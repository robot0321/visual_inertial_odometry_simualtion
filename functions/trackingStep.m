%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% <trackingStep.m specification>
% 1. From previous features, get the (new) current features
% 2. To use in consistency check, slidingWindowManager, etc., save the results in featGroup with various form
% 3. In real application, tracking will be done with other methods like KLT etc.
%    
% INPUT    : featGroup.feat_prevValidx, (previous/current)robotParams, cameraParams
% OUTPUT  : featGroup (including 'feat_intrscPrevIdx','feat_prevTrckPixel','feat_intrscCurrIdx','feat_currTrckPixel'
%                          'feat_currNewValidx','feat_currNewPixel','feat_prevDeadValidx','feat_intrsectValidx','feat_world_validx')
% FUNCTION: Getting featGroup from the previous feature and robotParams
% 
% Tuning Parameter: NONE
% 
% Copyright (c) 2019 JaeYoung Chung (robot0321@github) All Rights Reserved
% Lisence: GNU GENERAL PUBLIC LICENSE Version 3, 29 June 2007
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function featGroup = trackingStep(featGroup, robotParamsPrev, robotParams, cameraParams)
    % Feature Constraints in Current Step
    feat_world_validx = featureConstraint({'distance','heading','pixelRange'}, robotParams, cameraParams);
    feat_currValidx = feat_world_validx;

    % build Tracks of tracked (exist on both prev&curr) features 
    feat_prevDeadValidx = setdiff(featGroup.feat_prevValidx, feat_currValidx);
    feat_currNewValidx = setdiff(feat_currValidx, featGroup.feat_prevValidx);    % new features 
    feat_intrsectValidx = intersect(feat_currValidx, featGroup.feat_prevValidx); % features which are tracked 
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
    [feat_prevTrckPixel, feat_prevTrckNormal] = world2pixelNnormal_verFunc(feat_intrscPrevIdx, robotParamsPrev, cameraParams); % intersectValidx = featGroup.feat_prevValidx(:, feat_intrscPrevIdx)
    [feat_currTrckPixel, feat_currTrckNormal] = world2pixelNnormal_verFunc(feat_intrscCurrIdx, robotParams, cameraParams);      % intersectValidx = feat_currValidx(:, feat_intrscCurrIdx)
    [feat_currNewPixel,~]  = world2pixelNnormal_verFunc(feat_currNewValidx,  robotParams, cameraParams); 
    % caution: not-tracked previous features are not drew (because not interested)
    
    % Adding Pixel Error if needed
    if(cameraParams.errorParams.pixelErrParams.isCamPixelError)
        currNewPixelErr = cameraParams.errorParams.pixelErrParams.PixelErr * randn(size(feat_currNewPixel));
        currTrckPixelErr = cameraParams.errorParams.pixelErrParams.PixelErr * randn(size(feat_currTrckPixel));
        feat_currNewPixel = feat_currNewPixel + currNewPixelErr;
        feat_currTrckPixel = feat_currTrckPixel + currTrckPixelErr;
    end
    
    featGroup = struct('feat_intrscPrevIdx',feat_intrscPrevIdx, 'feat_prevTrckPixel', feat_prevTrckPixel, ...
                      'feat_intrscCurrIdx',feat_intrscCurrIdx, 'feat_currTrckPixel', feat_currTrckPixel, ...
                      'feat_currNewValidx',feat_currNewValidx, 'feat_currNewPixel', feat_currNewPixel, ...
                      'feat_prevDeadValidx',feat_prevDeadValidx, 'feat_intrsectValidx', feat_intrsectValidx, ...
                      'feat_world_validx', feat_world_validx, ...
                      'feat_prevTrckNormal',feat_prevTrckNormal, 'feat_currTrckNormal',feat_currTrckNormal);
end