% Feature Constraint
function world_valid_index = featureConstraint(featConstType, robotParams, cameraParams)
    world_valid_index = ones(1, size(robotParams.feat_position,2)); % the number of features
    currpos = - robotParams.Tbw(1:3,1:3)'*robotParams.Tbw(1:3,4);
    for i=1:length(featConstType)
        switch(featConstType{i})
            case('distance')
                % Distance Constraint
                feat_dist = sqrt(sum((robotParams.feat_position - currpos).^2, 1)); 
                feat_dist_validx = feat_dist>cameraParams.mindist & feat_dist<cameraParams.maxdist;
                world_valid_index = world_valid_index & feat_dist_validx;
            case('heading')
                % Camera Heading Constraint
                feat_cam = cameraParams.Tcb * robotParams.Tbw * [robotParams.feat_position(1:3,:); ones(1,size(robotParams.feat_position,2))];
                feat_cam_validx = feat_cam(3,:)>0; % camera attitude constraint
                world_valid_index = world_valid_index & feat_cam_validx;
            case('pixelRange')
                % Camera Intrinsic Matrix & Pixel Range Constraint
                [feat_pixel,~] = world2pixelNnormal_verFunc(1:size(robotParams.feat_position,2), robotParams, cameraParams);
                feat_pixel_validx = feat_pixel(1,:)>0 & feat_pixel(1,:)<cameraParams.px ...
                            & feat_pixel(2,:)>0 & feat_pixel(2,:)<cameraParams.py; 
                world_valid_index = world_valid_index & feat_pixel_validx;
            otherwise
                warning('Not proper options');
        end     
    end
    % The features with all the constriant in world frame
    world_valid_index = find(world_valid_index);
end