%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% <SIM6 specification>
% 1. visual system from SIM2 + IMU system from SIM3
% 2. 
% 3. 
% 
% Copyright with Jae Young Chung, robot0321 at github 
% Lisence: GNU GENERAL PUBLIC LICENSE Version 3, 29 June 2007
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc; clear; close all;
rng(10);

%% environment setting
Nfeatures = 1000; % the number of features
% x,y: 'Nfeatures' number of features, in range (-50,100)
feat_position = [rand(2,Nfeatures)*100 - 25; rand(1,Nfeatures)*30-15];

%% path setting
fwd_speed = 4; % speed along the forward direction
rot_time = 10; % steps needs to be rotated

% body position and attitude in world view
tw_bw  = [0:fwd_speed:50, 50*ones(1,rot_time), 50*ones(size(0:fwd_speed:50)), 50*ones(1,rot_time),  50:-fwd_speed:0, 0*ones(1,rot_time), zeros(size(50:-fwd_speed:0));
         zeros(size(0:fwd_speed:50)), 0*ones(1,rot_time), 0:fwd_speed:50, 50*ones(1,rot_time), 50*ones(size(50:-fwd_speed:0)), 50*ones(1,rot_time), 50:-fwd_speed:0;
         zeros(1, size(0:fwd_speed:50,2)+size(0:fwd_speed:50,2)+size(50:-fwd_speed:0,2)+size(50:-fwd_speed:0,2)+ rot_time*3)];
heading = pi/180*[zeros(size(0:fwd_speed:50)), linspace(0,90,rot_time), 90*ones(size(0:fwd_speed:50)), ...
            linspace(90,180,rot_time), 180*ones(size(50:-fwd_speed:0)),linspace(180,270,rot_time), 270*ones(size(50:-fwd_speed:0))]; % yaw (rad)
pitch = zeros(size(heading));
roll = pi*ones(size(heading));
Rbw = angle2dcm(heading, pitch, roll); % dcm rotation matrix

% Transformation matrix world to body T^w_b
tb_bw = zeros(3,1,size(tw_bw,2));
for i=1:size(tw_bw,2)
    tb_bw(:,1,i) = -Rbw(:,:,i)*tw_bw(:,i); 
end
Tbw = [Rbw, tb_bw; zeros(1,3,size(tw_bw,2)),ones(1,1,size(tw_bw,2))]; % T^b_w

%% camera setting
Tcb = [angle2dcm(pi/2, 0,pi/2), -[0;0;1e-3]; zeros(1,3), 1]; % T^c_b, extrinsic
% -> the origin of camera frame is at the (x = 1e-3m, y = 0, z = 0) in body frame
f = 433; % focal_length
cx = 320.5; cy=240.5; % 640 x 480 (arbitariliy selected)
K = [f, 0, cx;
     0, f, cy;
     0, 0, 1]; % intrinsic matrix

mindist = 2; % minimum distance from a feature to camera principal point
maxdist = 30; % maximum distance "

%% driving robot
for i=1:size(tw_bw,2)
    % Select constrainted features
    currpos = tw_bw(:,i);
    curratt = Rbw; 

    % distance constraint
    feat_dist = sqrt(sum((feat_position - currpos).^2, 1)); 
    feat_dist_validx = find(feat_dist>mindist & feat_dist<maxdist); 
    
    % camera attitude constraint
    feat_cam = Tcb*Tbw(:,:,i)*[feat_position(1:3,feat_dist_validx); ones(1,size(feat_dist_validx,2))];
    feat_cam_validx = feat_cam(3,:)>0; 
    
    % camera intrinsic matrix constraint
    feat_pixel = K*(feat_cam(1:3,feat_cam_validx)./feat_cam(3,feat_cam_validx));
    feat_pixel_validx = feat_pixel(1,:)>0 & feat_pixel(1,:)<640 ...
                        & feat_pixel(2,:)>0 & feat_pixel(2,:)<480; 
                    
    % The features with all the constriant in world frame                
    feat_world_validx = feat_dist_validx(feat_cam_validx); feat_world_validx = feat_world_validx(feat_pixel_validx);

    % index saving for optical flow
    k=i-1;
    if(i==1), feat_prevValidx = feat_world_validx; k=i; end
    feat_currValidx = feat_world_validx;

    figure(1); 
    % 3-D features, robot, path and the features in view
    subplot(1,2,1); 
    scatter3(feat_position(1,:), feat_position(2,:),feat_position(3,:),'b'); hold on;
    plot3(tw_bw(1,:), tw_bw(2,:), tw_bw(3,:),'r'); 
    plot3([currpos(1)*ones(1,size(feat_world_validx,2)); feat_position(1,feat_world_validx)], ...
         [currpos(2)*ones(1,size(feat_world_validx,2)); feat_position(2,feat_world_validx)], ...
         [currpos(3)*ones(1,size(feat_world_validx,2)); feat_position(3,feat_world_validx)],'k'); 
    scatter3(currpos(1),currpos(2), currpos(3), 50,'g','filled');
    axis equal; hold off;
    
    % Features projected on the camera with the constraints and optical flow
    subplot(1,2,2); 
    feat_intrsctValidx = intersect(feat_prevValidx, feat_currValidx); % feature which is tracked 
    feat_currNewPixel = world2cam(feat_position(:,setdiff(feat_currValidx, feat_prevValidx)), K, Tcb*Tbw(:,:,i)); % features which is newly appeared 
    feat_currTrckPixel = world2cam(feat_position(:,feat_intrsctValidx), K, Tcb*Tbw(:,:,i)); 
    feat_prevTrckPixel = world2cam(feat_position(:,feat_intrsctValidx), K, Tcb*Tbw(:,:,k));
    % caution: not-tracked previous features are not drew (because not interested)
    
    % current & previous features on camera image plane
    scatter(feat_currTrckPixel(1,:), feat_currTrckPixel(2,:),50,'r'); hold on;
    if ~isempty(feat_currNewPixel), scatter(feat_currNewPixel(1,:), feat_currNewPixel(2,:),50,'r'); hold on; end % avoid the error when it is empty
    scatter(feat_prevTrckPixel(1,:), feat_prevTrckPixel(2,:),'g'); 
    
    % optical flow between tracked features
    plot([feat_prevTrckPixel(1,:); feat_currTrckPixel(1,:)], [feat_prevTrckPixel(2,:); feat_currTrckPixel(2,:)],'-y');
    axis equal; hold off;
    axis([0, 640, 0, 480]);
    
    % For next step
    feat_prevValidx = feat_currValidx;
end


function pixel2 = world2cam(feat_world3, K, Tcw)
    % transformation from world to camera with intrinsic matrix 
    if isempty(feat_world3)
        pixel2 = []; return;
    end
    if (size(feat_world3,2)==3)
        feat_world3 = feat_world3';
    end
    feat_cam = Tcw*[feat_world3; ones(1,size(feat_world3,2))];
    feat_pixel = K*(feat_cam(1:3,:)./feat_cam(3,:));
    pixel2 = feat_pixel(1:2,:);
end