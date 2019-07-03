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
% Tuning Parameter: min/maxdist, PixelErr, DistanceThreshold(fundamental
% matrix), distCoeff(& error)
% 
% Copyright with Jae Young Chung, robot0321 at github 
% Lisence: GNU GENERAL PUBLIC LICENSE Version 3, 29 June 2007
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc; clear; close all;
rng(10);

%% environment setting
Nfeatures = 2000; % the number of features
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
isDistorted = false;
distCoeff = [-0.5359, 0.3669, -0.0035, 0.0073, 0.1043, -0.0387, -0.0127]; % example

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
    [feat_prevTrckPixel, feat_prevTrckNormal] = world2pixelNnormal(feat_position(:,feat_intrsectValidx), K, Tcb*Tbw(:,:,k), distCoeff, isDistorted);
    feat_currNewPixel                         = world2pixelNnormal(feat_position(:,feat_currNewValidx), K, Tcb*Tbw(:,:,i), distCoeff, isDistorted);  % features which is newly appeared 
    [feat_currTrckPixel, feat_currTrckNormal] = world2pixelNnormal(feat_position(:,feat_intrsectValidx), K, Tcb*Tbw(:,:,i), distCoeff, isDistorted);
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


%% functions 
function [pixel2, normal] = world2pixelNnormal(feat_world3, K, Tcw, distCoeff, isDistorted)
    % Transformation from world to camera with intrinsic matrix 
    if isempty(feat_world3)
        pixel2 = []; return;
    end
    if (size(feat_world3,2)==3)
        feat_world3 = feat_world3';
    end
    feat_camera4 = Tcw * [feat_world3; ones(1,size(feat_world3,2))];
    % [Xc, Yc, Zc] -- /Zc --> [Xc/Zc, Yc/Zc, 1] = [x_nu, y_nu, 1] (simulation)
    feat_normUndist = feat_camera4(1:3,:)./feat_camera4(3,:);
    
    % Error modeling due tothe lens
    distCoeff5 = distCoeff(1:7); % k1, k2, p1, p2, k3, k4, k5
    distCoeff2 = distCoeff(1:4); % k1, k2, p1, p2
    if(isDistorted)
        % [x_nu, y_nu, 1] -- LensDistortion(5th order) --> [x_nd, y_nd, 1]
        feat_normDistort3 = LensDistortion(feat_normUndist, distCoeff5); 
        
        % [x_nd, y_nd, 1] -- undistort(2nd order) --> [x_nu2, y_nu2, 1]
        feat_normUndist2 = undistort(feat_normDistort3, distCoeff2, 10);
        
        % [x_nu2, y_nu2, 1] -- K* --> [x_pu, y_pu, 1]
        normal = feat_normUndist2;
        feat_pixel = K*feat_normUndist2;
    else
        % [x_nu, y_nu, 1] -- K* --> [x_pu, y_pu, 1]
        normal = feat_normUndist;
        feat_pixel = K*feat_normUndist;
    end
    
    pixel2 = feat_pixel(1:2,:);
end

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