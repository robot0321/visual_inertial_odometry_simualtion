%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% <drawFigures.m specification>
% 1. Screening the valid index from user-selected constraint 
%    
% INPUT   : User selected constraint-type ('distance', 'heading', 'pixelRange', robotParams, cameraParams)
% OUTPUT  : World Index which fits to the constraint
% FUNCTION: Getting the valid (world) index from user-selected contraint 
% 
% Tuning Parameter: NONE
% 
% Copyright (c) 2019 JaeYoung Chung (robot0321@github) All Rights Reserved
% Lisence: GNU GENERAL PUBLIC LICENSE Version 3, 29 June 2007
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function drawFigures(traj_world_wb, feat_position, currStep, RSvalid_logic, TrackParams, featGroup, cameraParams)
    subx=2; suby=2;
    if(isfield(TrackParams,'reprodFeat')), subx=2; suby=3; end

    figure(1); 
    %% 3-D features, robot, path and the features in view
    subplot(subx,suby,1); 
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
    
    %% current & previous features on camera image plane
    subplot(subx,suby,2); 
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
    
    %% LiveTracks & DeadTracks
    subplot(subx,suby,4);
    for j = 1:length(TrackParams.LiveTracks)
        if(~isempty(TrackParams.LiveTracks{j})), LT=plot(TrackParams.LiveTracks{j}.pts(1,:), TrackParams.LiveTracks{j}.pts(2,:),'-og'); hold on; end
    end
    for j = 1:length(TrackParams.DeadTracks)
        if(~isempty(TrackParams.DeadTracks{j})), DT=plot(TrackParams.DeadTracks{j}.pts(1,:), TrackParams.DeadTracks{j}.pts(2,:), '-or'); hold on; end
    end
    hold off; 
    title('Tracks'); xlabel('x'); ylabel('y');
    axis([0, cameraParams.px, 0, cameraParams.py]);
    set(gca,'XAxisLocation','top','YAxisLocation','left','ydir','reverse')
    if exist('LT', 'var')&&~exist('DT', 'var'), legend(LT(1), 'LiveTrack'); end
    if exist('LT', 'var')&&exist('DT', 'var'),  legend([LT(1),DT(1)], 'LiveTrack','DeadTrack'); end
    
    %% 3D position reproduce error & projected tracks
    if isfield(TrackParams,'reprodFeat')
        subplot(subx,suby,3);
        normlist = []; idxlist=[];
        for ii=1:length(TrackParams.reprodFeat.feat3D)
            if ~isempty(TrackParams.reprodFeat.feat3D{ii})
                normlist = [normlist, norm(TrackParams.reprodFeat.feat3D{ii}-feat_position(:,ii))];
                idxlist = [idxlist, ii];
            end
        end
        stem(idxlist, normlist);
        title(['global 3D position error with mu: ', num2str(TrackParams.reprodFeat.mu)]);
        xlabel('index'); ylabel('norm2 err [m]')

        % 2D reprojection 
        subplot(subx,suby,6);
        for ii=1:length(TrackParams.reprodFeat.feat2D)
            if ~isempty(TrackParams.reprodFeat.feat2D{ii})
                plot(TrackParams.reprodFeat.feat2D{ii}.pts_true(1,:), TrackParams.reprodFeat.feat2D{ii}.pts_true(2,:),'-go','LineWidth',2); hold on
                plot(TrackParams.reprodFeat.feat2D{ii}.pts(1,:), TrackParams.reprodFeat.feat2D{ii}.pts(2,:),'-bo');
            end
        end
        hold off
        title(['Live Tracks with mu: ', num2str(TrackParams.reprodFeat.mu)]); xlabel('x'); ylabel('y');
        set(gca,'XAxisLocation','top','YAxisLocation','left','ydir','reverse');
        axis([0, cameraParams.px, 0, cameraParams.py]);
        legend('trueTracks', 'reproductTracks');
    end    
    drawnow();
end