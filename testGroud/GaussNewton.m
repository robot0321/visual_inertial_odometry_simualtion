clear;clc;close all;
load('tracks.mat');

for iter = 1:length(LiveTracks)
    if ~isempty(LiveTracks{iter})
        trackNum = iter;

        pCi_list=[];
        Tcic1 = zeros(4,4,length(LiveTracks{trackNum}.frame));
        % Tciw = zeros(4,4,length(LiveTracks{trackNum}.frame));
        Tc1w = cameraParams.Tcb*Tbw(:,:,LiveTracks{trackNum}.frame(1));
        for kk=1:LiveTracks{trackNum}.frame(end)-LiveTracks{trackNum}.frame(1)+1
            Tcw=cameraParams.Tcb*Tbw(:,:,kk);
            pCi_list = [pCi_list, Tcw(1:3,1:3)*feat_position(:,trackNum) + Tcw(1:3,4)];
            Tcic1(:,:,kk) = Tcw/Tc1w;
        end
        T12 = Tcic1(:,:,end);
        f_c1_0 = triangulateTwoView(T12, cameraParams.K, LiveTracks{trackNum}.pts(:,[1,end]));
        [f_c1, status, f_cis] = triangulateMultiView(Tcic1, f_c1_0, cameraParams.K, LiveTracks{trackNum}.pts, [1,0;0,1]); %모든 자세에 대한 multi view
        f_g2v = Tc1w\[f_c1_0; 1];f_g2v = f_g2v(1:3,1);  % feature의 글로벌g 위치 구하기
        f_gMulti = Tc1w\[f_c1; 1];f_gMulti = f_gMulti(1:3,1);  % feature의 글로벌g 위치 구하기

        fprintf("True posG: [%f, %f, %f] \n est2view posG: [%f, %f, %f] \n estMultiView posG: [%f, %f, %f]", ...
                    feat_position(1,trackNum),feat_position(2,trackNum),feat_position(3,trackNum),...
                    f_g2v(1), f_g2v(2), f_g2v(3), ...
                    f_gMulti(1),f_gMulti(2),f_gMulti(3));
        True_meas = cameraParams.K*(pCi_list./pCi_list(3,:)); True_meas = True_meas(1:2,:);
        estMulti_meas = cameraParams.K*(f_cis./f_cis(3,:)); estMulti_meas = estMulti_meas(1:2,:);

        figure(1);
        plot(LiveTracks{iter}.pts(1,:), LiveTracks{iter}.pts(2,:),'-go'); hold on;
        % plot(est2view_meas(1,:), est2view_meas(2,:),'-o'); hold on;
        plot(estMulti_meas(1,:), estMulti_meas(2,:),'-bo'); hold on;
        title('Live Tracks'); xlabel('x'); ylabel('y');
        set(gca,'XAxisLocation','top','YAxisLocation','left','ydir','reverse');
        axis([0, cameraParams.px, 0, cameraParams.py]);
    end
end

function f_c1 = triangulateTwoView(T12, K, pts)
    C12 = T12(1:3,1:3);			t12_1 = T12(1:3,4);
    obser = K\[pts; ones(1,2)]; % 1st person coordinate
    v_1 = obser(:,1);    	v_1 = v_1/norm(v_1);
    v_2 = obser(:,end);		v_2 = v_2/norm(v_2);
    A = [v_1 -C12*v_2];    	b = t12_1;
    lambda = A\b;
    f_c1 = lambda(1)*v_1;
end

function [f_c1, status, f_cis] = triangulateMultiView(Tset, f_c1_0, K, pts, R)
    % Gauss-Newton minimization with inverse-depth
    Ntracks = size(Tset,3);
    xHat = [f_c1_0(1:2,1)/f_c1_0(3); 1/f_c1_0(3)]; % f_c1_0는 일종의 초기값 역할을 한다.
    maxIter = 10;
    Cprev = Inf;

    for i = 1:maxIter
        A = zeros(2*Ntracks, 3);
        b = zeros(2*Ntracks, 1);
        W = zeros(2*Ntracks, 2*Ntracks);

        for j = 1:Ntracks
            Ti1 = inv(Tset(:,:,j));
            h = Ti1*[xHat(1:2,1); 1; xHat(3,1)]; 
            rId = 2*(j-1) + [1:2];
            W(rId,rId) = inv(R);
            zHat = K*[[h(1); h(2)]/h(3);1];
            b(rId,1) = pts(1:2, j) - zHat(1:2); % b는 meas pts와/ zHat은 예상값 xHat의 모양을 변화시킨것

            % Jacobian
            dh_dg = K(1:2,1:2)*[1/h(3), 0, -h(1)/h(3)^2; 0, 1/h(3), -h(2)/h(3)^2];
            dg_dx = Ti1(1:3,[1,2,4]);
            Ablock = dh_dg*dg_dx;
            A(rId, :) = Ablock;
        end

        % Gauss-Newton Update
        Cnew = 0.5*b'*W*b;   % Cost function
        AtA = A'*W*A; % (inv(J)*J)의 weight버전
        dx_star = (AtA + 1e-3*diag(diag(AtA)))\A'*W*b; %pinv(J)의 weight 버전
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
        f_cis(:,ii) = Tset(:,:,ii)*[xHat;1];
    end
    f_cis = f_cis(1:3,:);

    % calculate rotation compensated parallex
    parallex = zeros(1,Ntracks-1);
    for i = 2:Ntracks
        T_1i = Tset(:,:,i);
        T_1im1 = Tset(:,:,i-1);
        T_im1i = T_1im1\T_1i;
        rot_comp_pts = K*T_im1i(1:3,1:3)*(K\[pts(1:2,i);1]);
        parallex(i-1) = norm(pts(1:2,i-1) - rot_comp_pts(1:2)/rot_comp_pts(3));
    end

% 		if(1/xHat(3) < 2 ||  Cnew > Ntracks*5 || mean(parallex) < 5)
    if(1/xHat(3) < 2 ||  1/xHat(3) > 100 || Cnew > Ntracks*5)
        status = 1;
    end
end