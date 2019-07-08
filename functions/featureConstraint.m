% Feature Constraint

function world_valid_index = featureConstraint(featConstType, data)
    world_valid_index = ones(1, size(data.feat_position,2)); % the number of features
    for i=1:length(featConstType)
        switch(featConstType{i})
            case('distance')
                % Distance Constraint
                feat_dist = sqrt(sum((data.feat_position - data.currpos).^2, 1)); 
                feat_dist_validx = feat_dist>data.mindist & feat_dist<data.maxdist;
                world_valid_index = world_valid_index & feat_dist_validx;
            case('heading')
                % Camera Heading Constraint
                feat_cam = data.Tcw*[data.feat_position(1:3,:); ones(1,size(data.feat_position,2))];
                feat_cam_validx = feat_cam(3,:)>0; % camera attitude constraint
                world_valid_index = world_valid_index & feat_cam_validx;
            case('pixelRange')
                % Camera Intrinsic Matrix & Pixel Range Constraint
                [feat_pixel,~] = world2pixelNnormal(data.feat_position, data.K, data.Tcw, data.distortParams);
                feat_pixel_validx = feat_pixel(1,:)>0 & feat_pixel(1,:)<data.px ...
                            & feat_pixel(2,:)>0 & feat_pixel(2,:)<data.py; 
                world_valid_index = world_valid_index & feat_pixel_validx;
            otherwise
                warning('Not proper options');
        end     
    end
    % The features with all the constriant in world frame
    world_valid_index = find(world_valid_index);
end