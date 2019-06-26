clc; clear; close all;
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 2차원에서 각도, 거리 조건에 맞는 feature들 골라서 이동하면서 확인하기

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Nfeatures = 1000;

path  = [0:0.5:50, 50*ones(size(0:0.5:50)), 50:-0.5:0, zeros(size(50:-0.5:0));
         zeros(size(0:0.5:50)), 0:0.5:50, 50*ones(size(50:-0.5:0)),50:-0.5:0];
heading = pi/180*[zeros(size(0:0.5:50)), 90*ones(size(0:0.5:50)), ...
            180*ones(size(50:-0.5:0)), 270*ones(size(50:-0.5:0))];

robot = [path; heading]; %heading은 x으로 부터 각도
         
rng(10);% x,y:-50~100 범위에 Nfeatures 갯수 만큼 뿌리기
feat_position = [rand(2,Nfeatures)*100 - 25];

%%
theta = 60*pi/180; % 초점에서 이미지 외곽으로 각도
mindist = 2; % 최소 1미터 이상
maxdist = 30; % 최대 20미터만

for i=1:length(robot)
    % 조건에 맞는 feature 고르기
    currpos = robot(1:2,i);
    feat_angle = atan2(feat_position(2,:)-currpos(2), feat_position(1,:)-currpos(1)) - robot(3,i);
    feat_angle(feat_angle<-pi) = feat_angle(feat_angle<-pi) + 2*pi;
    feat_angle(feat_angle>pi) = feat_angle(feat_angle>pi) - 2*pi;
    feat_dist = sqrt(sum((feat_position - currpos).^2, 1));
    feat_inview_idx = (feat_angle<theta & feat_angle>-theta & ...
                    feat_dist>mindist & feat_dist<maxdist);
    feat_inview = feat_position(:,feat_inview_idx);
    
    figure(1);
    scatter(feat_position(1,:), feat_position(2,:),'b'); hold on;
    plot(path(1,:), path(2,:),'r'); 
    scatter(currpos(1),currpos(2), 100,'g');
    plot([currpos(1)*ones(1,size(feat_inview,2)); feat_inview(1,:)], ...
         [currpos(2)*ones(1,size(feat_inview,2)); feat_inview(2,:)], 'k'); hold off;
end
