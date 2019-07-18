%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% <cameraSettings.m specification>
% 1. Set the camera parameters (Tcb, focal length, principle point, pixel range, intrinsic matrix)
% 2. Set error parameters (pixel err, distortion, mistracking) including whether use or not 
%    
% INPUT   : distRange(which includes mindist and maxdist) to set in the cameraParams
% OUTPUT  : cameraParams including errorParams
% FUNCTION: Set the camera in/extrinsic parameters and error parameters
% 
% Tuning Parameter: all the parameters
%                  1) camera parameters (Tcb, fx, fy, cx, cy, px, py, K)
%                  2) error parameters (isCamPixelError, PixelErr, 
%                                     isDistorted, distCoeff, distortOrder, errorFactor
%                                     isMistracked, misTrackingRatio)
% 
% Copyright (c) 2019 JaeYoung Chung (robot0321@github) All Rights Reserved
% Lisence: GNU GENERAL PUBLIC LICENSE Version 3, 29 June 2007
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function cameraParams = cameraSettings(distRange, errorSetting)
    if nargin==1
        errorSetting = {};
    end
    %% Camera Setting Parameters
    Tcb = [angle2dcm(pi/2, 0,pi/2), -[0;0;1e-3]; zeros(1,3), 1]; % T^c_b, extrinsic
    % -> the origin of camera frame is at the (x = 1e-3m, y = 0, z = 0) in body frame
    fx = 1298.248; fy = 1305.601; % focal_length
    cx = 591.385;  cy = 417.372;  % principle point
    px = 1280;     py = 800;      % 1280 x 800 (option)
    K = [fx/2, 0, cx;
         0, fy/2, cy;
         0,    0,  1]; % intrinsic matrix
    
    %% Camera Image Errors 
    errorParams = struct(); % Collecting error Parameters
    
    % noise on camera plane (pixel noise) about 0.1 ~ 2
    isCamPixelError = false;
    PixelErr = 1; 
    pixelErrParams = struct('isCamPixelError',isCamPixelError,'PixelErr',PixelErr);
    errorParams.pixelErrParams = pixelErrParams;
                
    % Whether applying undistortion error//distCoeff: 
    % distortion error is applied at world2pixel() function
    isDistorted = false;
    distCoeff = [-0.535910, 0.366895, -0.003531, 0.007271, 0]; % example [k1, k2, p1, p2, k3]
    distortOrder = [4, 4]; % the order of distortion/undistortion(with error)
    errorFactor = [0.1, 0.072, 0.0007, 0.0014]; % at undistortion
    distortParams = struct('isDistorted', isDistorted, 'distCoeff', distCoeff, ...
                           'distortOrder', distortOrder, 'errorFactor',errorFactor);
    errorParams.distortParams = distortParams;
                
    % Mistracking ratio during tracking (like KLT mistracking)
    isMistracked = false;
    misTrackingRatio = 0.05;
    misTrackParams = struct('isMistracked',isMistracked,'mistrackingRatio',misTrackingRatio);
    errorParams.misTrackParams = misTrackParams;
    
    for idx=1:length(errorSetting)
        switch(errorSetting{idx})
            case 'pixelErr'
                isCamPixelError = true;
            case 'distortion'
                isDistorted = true;
            case 'mistrack'
                isMistracked = true;
        end
    end
   
    % Organizing total camera parameters
    cameraParams = struct('Tcb',Tcb,'fLx',fx,'fLy',fy,'cx',cx, 'cy',cy,'px',px,'py',py, ...
                        'mindist',distRange.mindist,'maxdist',distRange.maxdist, 'K',K, 'errorParams', errorParams);
end