%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% <world2pixelNnormal_verFunc.m specification>
% 1. Calculating the coordinate of the feauture in world frame from the camera parameters and current robot(body) status 
% 2. Apply or not camera lens distortion as the setting of the cameraParams.errorParams.distortParams
% 3. undistortion error is represented as the Gaussian form of distortParams.errorFactor
% 4. undistortion is done iteratively
% 
% INPUT   : feature position in world frame, robotParams, cameraParams
% OUTPUT  : the coordiante in image plane [x_pu, y_pu] and the coordinate in the normalized camera frame [x_nu, y_nu, 1]
% FUNCTION: Transfer the features from world frame to image plane and normalized camera frame
% 
% Tuning Parameter: NONE
% 
% Copyright (c) 2019 JaeYoung Chung (robot0321@github) All Rights Reserved
% Lisence: GNU GENERAL PUBLIC LICENSE Version 3, 29 June 2007
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% transformation from world frame to pixel frame
% world frame --T^c_w--> camera frame --./Z--> normal camera frame --intrinsic K--> pixel frame
% distortParams: struct('isDistorted', 'distCoeff', 'distortOrder','errorFactor');
% Assuming there is an error on estimating the distCoeff

function [pixel2, normal] = world2pixelNnormal_verFunc(feat_world3_valid_idx, robotParams, cameraParams)
    % Transformation from world to camera with intrinsic matrix
    % features are arraged as 3xN matrix
    feat_world3 = robotParams.feat_position(:, feat_world3_valid_idx);
    if isempty(feat_world3)
        pixel2 = []; normal = []; return;
    end
    
    % Transformation from world frame to camera frame
    feat_camera4 = cameraParams.Tcb * robotParams.Tbw * [feat_world3; ones(1,size(feat_world3,2))];
    
    % From camera frame to normalized camera frame
    % [Xc, Yc, Zc] -- /Zc --> [Xc/Zc, Yc/Zc, 1] = [x_nu, y_nu, 1] 
    feat_normUndist = feat_camera4(1:3,:)./feat_camera4(3,:);

    
    % Error modeling due tothe lens
    distortParams = cameraParams.errorParams.distortParams;
    distCoeff_Dist = distortParams.distCoeff(1:distortParams.distortOrder(1)); % k1, k2, p1, p2, k3, k4, k5
    distCoeff_Undist = distortParams.distCoeff(1:distortParams.distortOrder(2)) ... % k1, k2, p1, p2
                        + distortParams.errorFactor.*randn(1,distortParams.distortOrder(2)); % adding Gaussian estimation Error
    if(distortParams.isDistorted) % Distortion/Undistortion model 
        % [x_nu, y_nu, 1] -- LensDistortion(5th order) --> [x_nd, y_nd, 1]
        feat_normDistort3 = LensDistortion(feat_normUndist, distCoeff_Dist); 
        
        % [x_nd, y_nd, 1] -- undistort(2nd order) --> [x_nu2, y_nu2, 1]
        feat_normUndist2 = undistort(feat_normDistort3, distCoeff_Undist, 10);
        
        % From normalized camera frame to pixel frame 
        % [x_nu2, y_nu2, 1] -- K* --> [x_pu, y_pu, 1]
        normal = feat_normUndist2;
        feat_pixel = cameraParams.K * feat_normUndist2;
    else
        % From normalized camera frame to pixel frame 
        % [x_nu, y_nu, 1] -- K* --> [x_pu, y_pu, 1]
        normal = feat_normUndist;
        feat_pixel = cameraParams.K * feat_normUndist;
    end

    pixel2 = feat_pixel(1:2,:);
end

%% Lens distortion model & undistortion 
function feat_normDistort3 = LensDistortion(feat_normUndist3, distCoeff5) % 5th order modeling
    dc = zeros(1,10); dc(1:length(distCoeff5)) = distCoeff5; % k1~5, p1,2
    
    x_nu = feat_normUndist3(1,:); y_nu = feat_normUndist3(2,:);
    r2 = x_nu.^2 + y_nu.^2;
    xy_nd = (1 + ((((dc(7).*r2 + dc(6)).*r2 + dc(5)).*r2 + dc(2)).*r2+ dc(1)).*r2) .* [x_nu;y_nu] ...
             + [2*dc(3)*x_nu.*y_nu + dc(4)*(r2+2*x_nu.*x_nu); dc(3)*(r2+2*y_nu.*y_nu) + 2*dc(4)*x_nu.*y_nu];
    feat_normDistort3 = [xy_nd; ones(1,size(xy_nd,2))];
end

function feat_normUndistort3 = undistort(feat_normDistorted3, distCoeff, maxIter) %2nd order approximation
    Nframe= size(feat_normDistorted3,2);
    dc = zeros(1,10); dc(1:length(distCoeff)) = distCoeff; % k1~5, p1,2

    % undistorted points
    x= feat_normDistorted3(1,:); y = feat_normDistorted3(2,:);
    x0 = x; y0 = y;
    for i = 1:maxIter
        r2 = x.^2 + y.^2;
        icdist =  1./(1 + ((((dc(7).*r2 + dc(6)).*r2 + dc(5)).*r2 + dc(2)).*r2+ dc(1)).*r2);
        dx = 2*dc(3)*x.*y + dc(4)*(r2 + 2*x.*x);
        dy = dc(3) * (r2 + 2*y.*y) + 2*dc(4)*x.*y;
        x = (x0-dx).*icdist;		y = (y0-dy).*icdist;
    end
    feat_normUndistort3 = [x; y; ones(1, Nframe)];
end