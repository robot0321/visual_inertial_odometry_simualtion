%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% <SIM1 specification>
% In 2-D, Features with angle and distance constraint
% 
% Copyright with Jae Young Chung, robot0321 at github 
% Lisence: MIT
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc; clear; close all; 
rng(10);

%% environment setting
Nfeatures = 500; % the number of features
% x,y: 'Nfeatures' number of features, in range (-50,100)
feat_position = rand(2,Nfeatures)*100 - 25;

%% path setting
fwd_speed = 2; % speed along the forward direction
rot_time = 10; % steps needs to be rotated

% path and heading of robot, heading is the angle from world-x axis
path  = [0:fwd_speed:50, 50*ones(1,rot_time), 50*ones(size(0:fwd_speed:50)), 50*ones(1,rot_time),  50:-fwd_speed:0, 0*ones(1,rot_time), zeros(size(50:-fwd_speed:0));
         zeros(size(0:fwd_speed:50)), 0*ones(1,rot_time), 0:fwd_speed:50, 50*ones(1,rot_time), 50*ones(size(50:-fwd_speed:0)), 50*ones(1,rot_time), 50:-fwd_speed:0];
heading = pi/180*[zeros(size(0:fwd_speed:50)), linspace(0,90,rot_time), 90*ones(size(0:fwd_speed:50)), ...
            linspace(90,180,rot_time), 180*ones(size(50:-fwd_speed:0)),linspace(180,270,rot_time), 270*ones(size(50:-fwd_speed:0))]; % yaw (rad)
robot = [path; heading]; 
         
%% camera setting
theta = 45*pi/180; % angle between principal axis and the edge of the image plane (arbitarilliy selected)
mindist = 2; % minimum distance from a feature to camera principal point
maxdist = 20; % maximum distance "

%% driving robot
for i=1:length(robot)
    % Select constrainted features
    currpos = robot(1:2,i);
    currhead = robot(3,i);
    
    % the angle from the principal axis of camera to a line between a feature and camera
    feat_angle = atan2(feat_position(2,:)-currpos(2), feat_position(1,:)-currpos(1)) - currhead;
    % compensation due to the uncontinuity of angle from atan2 (-pi ~ pi)
    feat_angle(feat_angle<-pi) = feat_angle(feat_angle<-pi) + 2*pi;
    feat_angle(feat_angle>pi) = feat_angle(feat_angle>pi) - 2*pi;
    
    % distance constraint 
    feat_dist = sqrt(sum((feat_position - currpos).^2, 1));
    
    % select features with angle and distance constraint
    feat_inview_idx = (feat_angle<theta & feat_angle>-theta & ...
                    feat_dist>mindist & feat_dist<maxdist);
    feat_inview = feat_position(:,feat_inview_idx);
    
    % draw features, path, robot, and view-line in 2-D
    figure(1);
    scatter(feat_position(1,:), feat_position(2,:),'b'); hold on; %features
    plot(path(1,:), path(2,:),'r'); % path
    scatter(currpos(1),currpos(2), 72,'g','filled'); % robot
    plot([currpos(1)*ones(1,size(feat_inview,2)); feat_inview(1,:)], ... 
         [currpos(2)*ones(1,size(feat_inview,2)); feat_inview(2,:)], 'k'); % f in view 
    hold off;
end
