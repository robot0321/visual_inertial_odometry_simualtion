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
% 
% Tuning Parameter: min/maxdist
% 
% Copyright with Jae Young Chung, robot0321 at github 
% Lisence: GNU GENERAL PUBLIC LICENSE Version 3, 29 June 2007
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc; clear; close all;
rng(10);

%% environment setting
Nfeatures = 500; % the number of features
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

%% driving robot
for i=1:size(tw_wb,2)
    % Current Robot Position and Attitute in World Frame 
    currpos = tw_wb(:,i); % t^w_bw: 
    curratt = Rbw; 
    
    %% Feature Constraints
    % Distance Constraint
    feat_dist = sqrt(sum((feat_position - currpos).^2, 1)); 
    feat_dist_validx = feat_dist>mindist & feat_dist<maxdist; 
    
    % Camera Heading Constraint
    feat_cam = Tcb*Tbw(:,:,i)*[feat_position(1:3,:); ones(1,size(feat_position,2))];
    feat_cam_validx = feat_cam(3,:)>0; % camera attitude constraint
    
    % Camera Intrinsic Matrix & Pixel Range Constraint
    feat_pixel = world2pixel(feat_position, K, Tcb*Tbw(:,:,i));
    feat_pixel_validx = feat_pixel(1,:)>0 & feat_pixel(1,:)<px ...
                        & feat_pixel(2,:)>0 & feat_pixel(2,:)<py; 
                    
    % The features with all the constriant in world frame                
    feat_world_validx = find(feat_dist_validx & feat_cam_validx & feat_pixel_validx);

    %% Feature Matching & Optical Flow
    % Index saving for optical flow
    k=i-1;
    if(i==1), feat_prevValidx = feat_world_validx; k=i; end
    feat_currValidx = feat_world_validx;
    
    % build Tracks of tracked (exist on both prev&curr) features 
    feat_currNewValidx = setdiff(feat_currValidx, feat_prevValidx);    % new features 
    feat_intrsectValidx = intersect(feat_currValidx, feat_prevValidx); % features which are tracked 

    % Features on Camera Plane 
    feat_prevTrckPixel = world2pixel(feat_position(:,feat_intrsectValidx), K, Tcb*Tbw(:,:,k));
    feat_currTrckPixel = world2pixel(feat_position(:,feat_intrsectValidx), K, Tcb*Tbw(:,:,i));
    feat_currNewPixel  = world2pixel(feat_position(:,feat_currNewValidx), K, Tcb*Tbw(:,:,i)); 
    % caution: not-tracked previous features are not drew (because not interested)
    
    %% draw figures
    figure(1); 
    % 3-D features, robot, path and the features in view
    subplot(1,2,1); 
    scatter3(feat_position(1,:), feat_position(2,:),feat_position(3,:),'b'); hold on; %features
    plot3(tw_wb(1,:), tw_wb(2,:), tw_wb(3,:),'r'); % path
    plot3([currpos(1)*ones(1,size(feat_world_validx,2)); feat_position(1,feat_world_validx)], ...
         [currpos(2)*ones(1,size(feat_world_validx,2)); feat_position(2,feat_world_validx)], ...
         [currpos(3)*ones(1,size(feat_world_validx,2)); feat_position(3,feat_world_validx)],'k'); % view ray
    scatter3(currpos(1),currpos(2), currpos(3), 50,'g','filled'); % robot
    axis equal; hold off;
    
    subplot(1,2,2); 
    % current & previous features on camera image plane
    if ~isempty(feat_intrsectValidx) % avoid the error when it is empty
        scatter(feat_currTrckPixel(1,:), feat_currTrckPixel(2,:),50,'r'); hold on;
        scatter(feat_prevTrckPixel(1,:), feat_prevTrckPixel(2,:),'g');
        plot([feat_prevTrckPixel(1,:); feat_currTrckPixel(1,:)], [feat_prevTrckPixel(2,:); feat_currTrckPixel(2,:)],'-y');
    end
    if ~isempty(feat_currNewValidx), scatter(feat_currNewPixel(1,:), feat_currNewPixel(2,:),50,'r'); end 
    
    % optical flow between tracked features
    axis equal; hold off;
    axis([0, 640, 0, 480]);
    
%% For next step
    feat_prevValidx = feat_currValidx;
end


%% functions 
function pixel2 = world2pixel(feat_world3, K, Tcw)
    % Transformation from world to camera with intrinsic matrix 
    if isempty(feat_world3)
        pixel2 = []; return;
    end
    if (size(feat_world3,2)==3)
        feat_world3 = feat_world3';
    end
    
    % Transformation from world frame to camera frame
    feat_camera4 = Tcw * [feat_world3; ones(1,size(feat_world3,2))];
    
    % From camera frame to normalized camera frame
    % [Xc, Yc, Zc] -- /Zc --> [Xc/Zc, Yc/Zc, 1] = [x_nu, y_nu, 1] 
    feat_normUndist = feat_camera4(1:3,:)./feat_camera4(3,:);

    % From normalized camera frame to pixel frame 
    % [x_nu, y_nu, 1] -- K* --> [x_pu, y_pu, 1]
    feat_pixel = K*feat_normUndist;

    pixel2 = feat_pixel(1:2,:);
end
