%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% <drawFigures.m specification>
% 1. Screening the valid index from user-selected constraint 
%    
% INPUT    : User selected constraint-type ('distance', 'heading', 'pixelRange', robotParams, cameraParams)
% OUTPUT  : World Index which fits to the constraint
% FUNCTION: Getting the valid (world) index from user-selected contraint 
% 
% Tuning Parameter: NONE
% 
% Copyright (c) 2019 JaeYoung Chung (robot0321@github) All Rights Reserved
% Lisence: GNU GENERAL PUBLIC LICENSE Version 3, 29 June 2007
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function drawFigures(traj_world_wb, feat_position, LiveTracks, DeadTracks, currStep, RSvalid_logic, featGroup, cameraParams)
    figure(1); 
    % 3-D features, robot, path and the features in view
    subplot(2,2,1); 
    scatter3(feat_position(1,:), feat_position(2,:),feat_position(3,:),'b'); hold on; %features
    plot3(traj_world_wb(1,:), traj_world_wb(2,:), traj_world_wb(3,:),'r'); % path
    plot3([traj_world_wb(1,currStep)*ones(1,size(featGroup.feat_world_validx,2)); feat_position(1,featGroup.feat_world_validx)], ...
         [traj_world_wb(2,currStep)*ones(1,size(featGroup.feat_world_validx,2)); feat_position(2,featGroup.feat_world_validx)], ...
         [traj_world_wb(3,currStep)*ones(1,size(featGroup.feat_world_validx,2)); feat_position(3,featGroup.feat_world_validx)],'k'); % view ray
    scatter3(traj_world_wb(1,currStep),traj_world_wb(2,currStep),traj_world_wb(3,currStep), 50,'g','filled'); % robot position
%     yaw = dcm2angle(Tbw(1:3,1:3,currStep));
%     quiver3(traj_world_wb(1,currStep),traj_world_wb(2,currStep),traj_world_wb(3,currStep), ...
%              3*cos(yaw*pi/180),3*sin(yaw*pi/180),yaw*0); % robot heading
    axis equal; grid on; hold off; xlabel('x'); ylabel('y');
    
    subplot(2,2,2); 
    % current & previous features on camera image plane
    if ~isempty(featGroup.feat_intrsectValidx) % avoid the error when it is empty
        scatter(featGroup.feat_currTrckPixel(1,:), featGroup.feat_currTrckPixel(2,:),50,'r'); hold on;
        scatter(featGroup.feat_prevTrckPixel(1,:), featGroup.feat_prevTrckPixel(2,:),'g');
        % draw the valid(yellow) and invalid(black) optical flows
        plot([featGroup.feat_prevTrckPixel(1,RSvalid_logic); featGroup.feat_currTrckPixel(1,RSvalid_logic)], [featGroup.feat_prevTrckPixel(2,RSvalid_logic); featGroup.feat_currTrckPixel(2,RSvalid_logic)],'-y');
        plot([featGroup.feat_prevTrckPixel(1,~RSvalid_logic); featGroup.feat_currTrckPixel(1,~RSvalid_logic)], [featGroup.feat_prevTrckPixel(2,~RSvalid_logic); featGroup.feat_currTrckPixel(2,~RSvalid_logic)],'-k');
    end
    if ~isempty(featGroup.feat_currNewValidx), scatter(featGroup.feat_currNewPixel(1,:), featGroup.feat_currNewPixel(2,:),50,'r'); end 
    % optical flow between tracked features
    hold off; xlabel('x'); ylabel('y');
    axis([0, cameraParams.px, 0, cameraParams.py]);
    set(gca,'XAxisLocation','top','YAxisLocation','left','ydir','reverse');
    
    % LiveTracks
    subplot(2,2,3);
    for j = 1:length(LiveTracks)
        if(~isempty(LiveTracks{j})), plot(LiveTracks{j}.pts(1,:), LiveTracks{j}.pts(2,:),'-og'); hold on; end
    end
    title('Live Tracks'); xlabel('x'); ylabel('y');
    hold off; axis([0, cameraParams.px, 0, cameraParams.py]);
    set(gca,'XAxisLocation','top','YAxisLocation','left','ydir','reverse');
    
    % DeadTracks
    subplot(2,2,4);
    for j = 1:length(DeadTracks)
        if(~isempty(DeadTracks{j})), plot(DeadTracks{j}.pts(1,:), DeadTracks{j}.pts(2,:), '-or'); hold on; end
    end
    title('Dead Tracks');
    hold off; axis([0, cameraParams.px, 0, cameraParams.py]);
    set(gca,'XAxisLocation','top','YAxisLocation','left','ydir','reverse');
    
    drawnow();
end