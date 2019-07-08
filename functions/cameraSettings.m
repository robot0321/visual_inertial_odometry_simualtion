function cameraParams = cameraSettings()
    %% Camera Setting Parameters
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

    %% Camera Image Errors 
    % noise on camera plane (pixel noise) about 0.1 ~ 2
    isCamPixelError = false;
    PixelErr = 1; 
    pixelErrParams = struct('isCamPixelError',isCamPixelError,'PixelErr',PixelErr);

    % Whether applying undistortion error//distCoeff: 
    % distortion error is applied at world2pixel() function
    isDistorted = false;
    distCoeff = [-0.5359, 0.3669, -0.0035, 0.0073, 0]; % example [k1, k2, p1, p2, k3]
    distortOrder = [4, 4]; % the order of distortion/undistortion(with error)
    errorFactor = [0.1, 0.072, 0.0007, 0.0014]; % at undistortion
    distortParams = struct('isDistorted', isDistorted, 'distCoeff', distCoeff, ...
                           'distortOrder', distortOrder, 'errorFactor',errorFactor);

    % Miss-tracking ratio during tracking (like KLT miss-tracking)
    isMisstracked = true;
    misTrackingRatio = 0.05;
    misTrackParams = struct('isMisstracked',isMisstracked,'misTrackingRatio',misTrackingRatio);

    % Collecting error Parameters
    errorParams = struct('pixelErrParams',pixelErrParams,'distortParams',distortParams,'misTrackParams',misTrackParams);
    
    % Organizing total camera parameters
    cameraParams = struct('Tcb',Tcb, 'focalLength',f, 'cx',cx, 'cy',cy,'px',px,'py',py, ...
                        'mindist',mindist,'maxdist',maxdist, 'K',K, 'errorParams', errorParams);
end