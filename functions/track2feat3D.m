%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% <track2feat3D.m specification>
% 1. Screening the valid index from user-selected constraint 
%    
% INPUT   : Tracks, position/attitude of body(Tbw), Levenberg-Marquardt tunning parameter(mu)
%           
% OUTPUT  : World Index which fits to the constraint
% FUNCTION: Getting the valid (world) index from user-selected contraint 
% 
% Tuning Parameter: NONE
% 
% Copyright (c) 2019 JaeYoung Chung (robot0321@github) All Rights Reserved
% Lisence: GNU GENERAL PUBLIC LICENSE Version 3, 29 June 2007
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function reprodFeat = track2feat3D(Tracks, Tbw, mu, robotParams, cameraParams)
%     persistent outnumline
    feat2D = cell(1, length(Tracks));
    feat3D = cell(1, length(Tracks));
    status = cell(1, length(Tracks));
    for trackNum = 1:length(Tracks)
        if(~isempty(Tracks{trackNum}) && length(Tracks{trackNum}.frame)~=1)
            % Getting the 3D & 2D(image plane) feature position from the LiveTrack

            pCi_list=[];
            Tcic1 = zeros(4,4,length(Tracks{trackNum}.frame)); % T^ci_c1 (different notation)
            Tc1w = cameraParams.Tcb*Tbw(:,:,Tracks{trackNum}.frame(1)); % T^c1_w
            for kk=Tracks{trackNum}.frame(1):Tracks{trackNum}.frame(end)
                Tcw=cameraParams.Tcb*Tbw(:,:,kk); % T^c_w = T^c_b * T^b_w
                pCi_list = [pCi_list, Tcw(1:3,1:3)*robotParams.feat_position(:,trackNum) + Tcw(1:3,4)]; % p^c = R^c_w * p^w + p^c_cw
                Tcic1(:,:,kk - Tracks{trackNum}.frame(1)+1) = Tcw/Tc1w; % T^ci_c1 = T^ci_w * inv(T^c1_w)
            end

            % Approximate 2-view solution as the start point of Multi-view solution
            % With the first and last frame of each LiveTrack, the baseline could be maximum in each LiveTrack
            Tc1cn = inv(Tcic1(:,:,end));
            f_c1_0 = triangulateTwoView(Tc1cn, cameraParams.K, Tracks{trackNum}.pts(:,[1,end]));
            if (size(Tracks{trackNum}.pts,2)>4)
                triangulateThreeView(Tcic1, cameraParams.K, Tracks{trackNum}.pts(:,1:end));
            end
            [f_c1Mv, ~, f_cisFMv] = triangulateMultiView(Tcic1, f_c1_0, cameraParams.K, Tracks{trackNum}.pts, [1,0;0,1], mu);
            % below operations are done for the trust test
            f_c1Mv22 = triangulateTwoView(inv(Tcic1(:,:,2)), cameraParams.K, Tracks{trackNum}.pts(:,1:2));
            [f_c1Mv33, ~, ~] = triangulateMultiView(Tcic1(:,:,[1,end]), f_c1_0, cameraParams.K, Tracks{trackNum}.pts(:,[1,end]), [1,0;0,1], mu);
            Tcn_1cn = Tcic1(:,:,end-1)/Tcic1(:,:,end);
            f_c1Mv44 = triangulateTwoView(Tcn_1cn, cameraParams.K, Tracks{trackNum}.pts(:,end-1:end));
            
            % The estimation of 3D (world) feature position
            f_g2v = Tc1w\[f_c1Mv; 1];                        % c1~cn (multi)
            f_g2v22 = Tc1w\[f_c1Mv22; 1];                    % c1 c2 (2)
            f_g2v33 = Tc1w\[f_c1Mv33; 1];                    % c1, cn (multi)
            f_g2v44 = Tc1w\(Tcic1(:,:,end-1)\[f_c1Mv44; 1]); % cn-1, cn (2)

            % trust test: consistency check between first 2-view, last 2-veiw, and so on...
            crit = 0.2;
%             tttr=[robotParams.feat_position(:,trackNum);1];
            if norm(f_g2v22-f_g2v44) > crit*norm(f_g2v)% || norm(f_g2v-f_g2v22)>crit || norm(f_g2v-f_g2v33)>crit
%                 [norm(tttr-f_g2v),norm(tttr-f_g2v22),norm(tttr-f_g2v33),norm(tttr-f_g2v44)]
%                 [tttr, f_g2v, f_g2v22, f_g2v33, f_g2v44]
%                 %true, multi, 1,2view, 1,nview, n-1,nview
%                 Tracks{trackNum}.world_idx
%                 outnumline = [outnumline, trackNum];
                continue; % do not save in feat3D, feat2D, status with current trackNum
            end
            
            feat3D{trackNum} = f_g2v(1:3,1);  % Global position of features with 2-view solution
            
            % Convertion from estimated camera frame coordinate to the image plane
            estMultiFMv_meas = cameraParams.K*(f_cisFMv./f_cisFMv(3,:));
            feat2D{trackNum} = struct('frame', Tracks{trackNum}.frame, 'pts', estMultiFMv_meas(1:2,:));
            True_meas = cameraParams.K*(pCi_list./pCi_list(3,:)); True_meas = True_meas(1:2,:);
            feat2D{trackNum}.pts_true = True_meas;
            
            % if the feature is stationary, make status=1
            status{trackNum} = 1;
        end
    end
    reprodFeat = struct(); reprodFeat.feat3D=feat3D; reprodFeat.feat2D=feat2D; reprodFeat.mu=mu; reprodFeat.status=status;
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

function f_c1 = triangulateThreeView(Tcisc1, K, pts)
%  it can be calculate N-view solution as this way but inverse of 3(N-1)*N matrix is not affordable
    Tc1cis = zeros(4,4,size(Tcisc1,3));
    Tc1cis(:,:,2) = inv(Tcisc1(:,:,2));
    Tc1cis(:,:,3) = inv(Tcisc1(:,:,3));
    
    C12 = Tc1cis(1:3,1:3,2); t12_1 = Tc1cis(1:3,4,2);
    C13 = Tc1cis(1:3,1:3,3); t13_1 = Tc1cis(1:3,4,3);
    obser = K\[pts; ones(1,size(pts,2))]; % 1st person coordinate
    v_1 = obser(:,1);    	v_1 = v_1/norm(v_1);
    v_2 = obser(:,2);		v_2 = v_2/norm(v_2);
    v_3 = obser(:,3);		v_3 = v_3/norm(v_3);
    A = [v_1, -C12*v_2, zeros(3,1); v_1, zeros(3,1), -C13*v_3];   b = [t12_1; t13_1];
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

    % f_c1: feature position in frame C1
    % f_cis: feature positions in frame C1~Cn
    f_c1 = [xHat(1:2)/xHat(3); 1/xHat(3)];
    status  = 0;
    f_cis = zeros(4, size(Tset,3));
    for ii=1:size(Tset,3)
        f_cis(:,ii) = Tset(:,:,ii)*[f_c1;1];
    end
    f_cis = f_cis(1:3,:);
end
