function feat_position = featureGeneration(traj_world_wb, featGenParams)
    Nfeatures = featGenParams.Nfeatures;
    step = featGenParams.step;
    xyRange = featGenParams.xyRange;
    zRange = featGenParams.zRange;
    
    pathL = floor(length(traj_world_wb)/step);
    NfeatPerL = ceil(Nfeatures/pathL);
    feat_position = [];
    for i = 1:pathL
        feat_position = [feat_position, traj_world_wb(:,step*(i-1)+1) + [(rand(2,NfeatPerL)-0.5)*xyRange; (rand(1,NfeatPerL)-0.5)*zRange]];
    end
    selectIdx = randperm(length(feat_position), Nfeatures);
    feat_position = feat_position(:,selectIdx);
end