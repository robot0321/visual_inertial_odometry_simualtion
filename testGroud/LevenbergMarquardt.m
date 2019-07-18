clear;clc;close all;
load('tracks.mat');

mulist = [1e-1, 1e0, 1e2, 1e3, 1e4, 1e6];
for muiter = 1:length(mulist)
mu = mulist(muiter);
for iter = 1:length(LiveTracks)
    if ~isempty(LiveTracks{iter})
        % Getting the 3D & 2D(image plane) feature position from the LiveTrack
        trackNum = iter;

        pCi_list=[];
        Tcic1 = zeros(4,4,length(LiveTracks{trackNum}.frame)); % T^ci_c1 (different notation)
        Tc1w = cameraParams.Tcb*Tbw(:,:,LiveTracks{trackNum}.frame(1)); % T^c1_w
        for kk=LiveTracks{trackNum}.frame(1):LiveTracks{trackNum}.frame(end)
            Tcw=cameraParams.Tcb*Tbw(:,:,kk); % T^c_w = T^c_b * T^b_w
            pCi_list = [pCi_list, Tcw(1:3,1:3)*feat_position(:,trackNum) + Tcw(1:3,4)]; % p^c = R^c_w * p^w + p^c_cw
            Tcic1(:,:,kk - LiveTracks{trackNum}.frame(1)+1) = Tcw/Tc1w; % T^ci_c1 = T^ci_w * inv(T^c1_w)
        end
        
        % Approximate 2-view solution as the start point of Multi-view solution
        % With the first and last frame of each LiveTrack, the baseline could be maximum in each LiveTrack
        Tc1cn = inv(Tcic1(:,:,end));
        f_c1_0 = triangulateTwoView(Tc1cn, cameraParams.K, LiveTracks{trackNum}.pts(:,[1,end]));
        [f_c1F2v, statusF2v, f_cisF2v] = triangulateMultiView(Tcic1, f_c1_0, cameraParams.K, LiveTracks{trackNum}.pts, [1,0;0,1], mu);
        
        % Use true solution as the start point of Multi-view solution
        f_true_c1_0 = Tc1w*[feat_position(:,trackNum); 1]; f_true_c1_0 = f_true_c1_0(1:3);
        [f_c1FTr, statusFTr, f_cisFTr] = triangulateMultiView(Tcic1, f_true_c1_0, cameraParams.K, LiveTracks{trackNum}.pts, [1,0;0,1], mu);
        
        %% The estimation of 3D feature position
        f_g2v = Tc1w\[f_c1_0; 1];f_g2v = f_g2v(1:3,1);  % Global position of features with 2-view solution
        f_gMulti = Tc1w\[f_c1FTr; 1];f_gMulti = f_gMulti(1:3,1);  % Global position of features with Multi-view solution
        fprintf("True posG: [%f, %f, %f] \n est2view posG: [%f, %f, %f] \n estMultiView posG: [%f, %f, %f] \n", ...
                    feat_position(1,trackNum),feat_position(2,trackNum),feat_position(3,trackNum),...
                    f_g2v(1), f_g2v(2), f_g2v(3), ...
                    f_gMulti(1),f_gMulti(2),f_gMulti(3));
                
        %% Convertion from estimated camera frame coordinate to the image plane
        True_meas = cameraParams.K*(pCi_list./pCi_list(3,:)); True_meas = True_meas(1:2,:);
        estMultiFTr_meas = cameraParams.K*(f_cisFTr./f_cisFTr(3,:)); estMultiFTr_meas = estMultiFTr_meas(1:2,:);
        estMultiF2v_meas = cameraParams.K*(f_cisF2v./f_cisF2v(3,:)); estMultiF2v_meas = estMultiF2v_meas(1:2,:);

        % Plot the estimated tracks on image plane 
        figure(1);
        subplot(2,3,muiter);
        plot(True_meas(1,:), True_meas(2,:),'-go','LineWidth',4); hold on;
        plot(estMultiFTr_meas(1,:), estMultiFTr_meas(2,:),'-bo','LineWidth',2); hold on;
        plot(estMultiF2v_meas(1,:), estMultiF2v_meas(2,:),'-ro'); hold on;
        title(['Live Tracks with mu: ', num2str(mu)]); xlabel('x'); ylabel('y');
        set(gca,'XAxisLocation','top','YAxisLocation','left','ydir','reverse');
        axis([0, cameraParams.px, 0, cameraParams.py]);
        legend('trueTracks', 'estTrack@true', 'estTrack@2veiw');
    end
end
end

%% function
% triangulation with epipolar constraint
function f_c1 = triangulateTwoView(Tc1cn, K, pts)
    C1n = Tc1cn(1:3,1:3);			t1n_1 = Tc1cn(1:3,4);
    obser = K\[pts; ones(1,2)]; % 1st person coordinate
    v_1 = obser(:,1);    	v_1 = v_1/norm(v_1);
    v_2 = obser(:,end);		v_2 = v_2/norm(v_2);
    A = [v_1 -C1n*v_2];    	b = t1n_1;
    lambda = A\b;
    f_c1 = lambda(1)*v_1;
end

function [f_c1, status, f_cis] = triangulateMultiView(Tset, f_c1_0, K, pts, R, mu)
    % Levenberg-Marquardt minimization with inverse-depth
    Ntracks = size(Tset,3);
    xHat = [f_c1_0(1:2,1)/f_c1_0(3); 1/f_c1_0(3)]; % f_c1_0 acts as a initial value of Multi-view iteration
    maxIter = 10;
    Cprev = Inf;

    %% Levenberg-Marquardt Update
    for i = 1:maxIter
        A = zeros(2*Ntracks, 3);
        b = zeros(2*Ntracks, 1);
        W = zeros(2*Ntracks, 2*Ntracks);

        for j = 1:Ntracks
            Ti1 = inv(Tset(:,:,j));
            h = Ti1*[xHat(1:2,1); 1; xHat(3,1)]; 
            rId = 2*(j-1) + [1:2];
            W(rId,rId) = inv(R);
            zHat = K*[[h(1); h(2)]/h(3);1]; % zHat: the projections of xHat on the each camera plane
            % Instead of 3D position in C1 frame, use the projected points on each frame, C1~Cn
            b(rId,1) = pts(1:2, j) - zHat(1:2); % b is the difference between pts(measured point) and zHat(xHat on image plane)
            
            % Jacobian
            dh_dg = K(1:2,1:2)*[1/h(3), 0, -h(1)/h(3)^2; 0, 1/h(3), -h(2)/h(3)^2];
            dg_dx = Ti1(1:3,[1,2,4]);
            Ablock = dh_dg*dg_dx;
            A(rId, :) = Ablock;
        end

        
        % if mu -> 0  , operate like Gauss-Newton 
        % if mu -> inf, operate like Gradient descent
        % In this noise-free case, mu over 10e6 shows a good result.
        Cnew = 0.5*b'*W*b;   % Cost function
        AtA = A'*W*A; % (inv(J)*J)의 weight버전
        dx_star = (AtA + mu*diag(diag(AtA)))\A'*W*b; %pinv(J)의 weight 버전
        xHat = xHat + dx_star;
        Cderiv = abs((Cnew - Cprev)/Cnew);
        Cprev = Cnew;

        if Cderiv < 1e-6
            break;
        end
    end

    f_c1 = [xHat(1:2)/xHat(3); 1/xHat(3)];
    status  = 0;
    f_cis = zeros(4, size(Tset,3));
    for ii=1:size(Tset,3)
        f_cis(:,ii) = Tset(:,:,ii)*[f_c1;1];
    end
    f_cis = f_cis(1:3,:);

%     % calculate rotation compensated parallex
%     parallex = zeros(1,Ntracks-1);
%     for i = 2:Ntracks
%         T_1i = Tset(:,:,i);
%         T_1im1 = Tset(:,:,i-1);
%         T_im1i = T_1im1\T_1i;
%         rot_comp_pts = K*T_im1i(1:3,1:3)*(K\[pts(1:2,i);1]);
%         parallex(i-1) = norm(pts(1:2,i-1) - rot_comp_pts(1:2)/rot_comp_pts(3));
%     end
% 
% % 		if(1/xHat(3) < 2 ||  Cnew > Ntracks*5 || mean(parallex) < 5)
%     if(1/xHat(3) < 2 ||  1/xHat(3) > 100 || Cnew > Ntracks*5)
%         status = 1;
%     end
end