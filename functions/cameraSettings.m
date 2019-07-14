function cameraParams = cameraSettings(distRange)
    %% Camera Setting Parameters
    Tcb = [angle2dcm(pi/2, 0,pi/2), -[0;0;1e-3]; zeros(1,3), 1]; % T^c_b, extrinsic
    % -> the origin of camera frame is at the (x = 1e-3m, y = 0, z = 0) in body frame
    fx = 1298.248; fy = 1305.601; % focal_length
    cx = 591.385;  cy = 417.372;  % principle point
    px = 1280;     py = 800;      % 1280 x 800 (option)
    K = [fx/2, 0, cx;
         0, fy/2, cy;
         0,    0,  1]; % intrinsic matrix

    %% Camera Image Errors 
    % noise on camera plane (pixel noise) about 0.1 ~ 2
    isCamPixelError = false;
    PixelErr = 1; 
    pixelErrParams = struct('isCamPixelError',isCamPixelError,'PixelErr',PixelErr);

    % Whether applying undistortion error//distCoeff: 
    % distortion error is applied at world2pixel() function
    isDistorted = false;
    distCoeff = [-0.535910, 0.366895, -0.003531, 0.007271, 0]; % example [k1, k2, p1, p2, k3]
    distortOrder = [4, 4]; % the order of distortion/undistortion(with error)
    errorFactor = [0.1, 0.072, 0.0007, 0.0014]; % at undistortion
    distortParams = struct('isDistorted', isDistorted, 'distCoeff', distCoeff, ...
                           'distortOrder', distortOrder, 'errorFactor',errorFactor);

    % Mistracking ratio during tracking (like KLT mistracking)
    isMistracked = false;
    misTrackingRatio = 0.05;
    misTrackParams = struct('isMistracked',isMistracked,'mistrackingRatio',misTrackingRatio);

    % Collecting error Parameters
    errorParams = struct('pixelErrParams',pixelErrParams,'distortParams',distortParams,'misTrackParams',misTrackParams);
    
    % Organizing total camera parameters
    cameraParams = struct('Tcb',Tcb,'fLx',fx,'fLy',fy,'cx',cx, 'cy',cy,'px',px,'py',py, ...
                        'mindist',distRange.mindist,'maxdist',distRange.maxdist, 'K',K, 'errorParams', errorParams);
end