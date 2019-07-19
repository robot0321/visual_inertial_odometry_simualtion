function d_list = epipolarConstraint(featGroup, robotParams, robotParamsPrev, cameraParams)
    % epipolar constraint (index k == b1 time / index i == b2 time)
    R_b1b2 = robotParamsPrev.Tbw(1:3,1:3) * robotParams.Tbw(1:3,1:3)'; % R_b1w * R_wb2
    dp_b1_b1b2 = robotParamsPrev.Tbw(1:3,4) - R_b1b2 * robotParams.Tbw(1:3,4); % Pb1_b1w - Rb1b2 * Pb2_b2w
    R = cameraParams.Tcb(1:3,1:3) * R_b1b2 * cameraParams.Tcb(1:3,1:3)'; % Rc1c2 = Rc1b1 * Rb1b2 * Rb2c2
    dp = cameraParams.Tcb(1:3,4) + cameraParams.Tcb(1:3,1:3)*dp_b1_b1b2 + (-R*cameraParams.Tcb(1:3,4));
    dp = dp/norm(dp);
    d_list = [];
    for j=1:length(featGroup.feat_prevTrckNormal)
        % epipolar constraint
        dpx = [0, -dp(3), dp(2); dp(3), 0, -dp(1); -dp(2), dp(1), 0];
        d_list = [d_list; featGroup.feat_prevTrckNormal(:,j)'* dpx * R * featGroup.feat_currTrckNormal(:,j)];
%         d_list = [d_list; dot(-cross(feat_prevTrckNormal(:,j), R*feat_currTrckNormal(:,j)), dp)];
    end
end