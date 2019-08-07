clear; close all; clc;
addpath('../functions/');

%%
NN = 300;
feat_position = [ones(2,NN); [(1:10)*0.1+1,(1:NN-10)+2]];
Tbw = zeros(4,4,2);
R = eye(3,3);
Tbw(:,:,1) = [R,zeros(3,1);zeros(1,3),1];
Tbw(:,:,2) = [R,[0;0;-1];zeros(1,3),1];

robotParams1 = struct('feat_position', feat_position, 'Tbw',Tbw(:,:,1));
robotParams2 = struct('feat_position', feat_position, 'Tbw',Tbw(:,:,2));
cameraParams = cameraSettings(struct('mindist',1, 'maxdist',100), {});
cameraParams.Tcb(1:3,1:3) = eye(3,3);


[feat1_TrckPixel, feat1_TrckNormal] = world2pixelNnormal_verFunc(1:size(feat_position,2), robotParams1, cameraParams); 
[feat2_TrckPixel, feat2_TrckNormal] = world2pixelNnormal_verFunc(1:size(feat_position,2), robotParams2, cameraParams); 
%%
varlist = []; meanlist = [];
for j=1:size(feat_position,2)
N = 500;
selectFeat = j; %1~10
errorlevel = 1e-3;
w = [1;1]*errorlevel;

fr3List = [];
for ii=1:N
    pts1 = feat1_TrckPixel(:,selectFeat) + w.*randn(2,1);
    pts2 = feat2_TrckPixel(:,selectFeat) + w.*randn(2,1);
    f_reprod3 = triangulateTwoView(Tbw(:,:,1)/Tbw(:,:,2), cameraParams.K, [pts1, pts2]);
    fr3List = [fr3List, f_reprod3];
end 

% scatter3(fr3List(1,:), fr3List(2,:), fr3List(3,:));

pca1 = pca(fr3List');
pcos = fr3List'*pca1(:,1)./vecnorm(fr3List)';
projpca = vecnorm(fr3List)'.*pcos;
meanlist = [meanlist, mean(projpca)];
varlist = [varlist, var(projpca)];
% figure(2); histogram(projpca,91); hold on;
end
% legend();title(['triangulation2view with Gaussian noise ',num2str(errorlevel)]);
%%
function f_c1 = triangulateTwoView(T12, K, pts)
    C12 = T12(1:3,1:3);			t12_1 = T12(1:3,4);
    obser = K\[pts; ones(1,2)]; % 1st person coordinate
    v_1 = obser(:,1);    	v_1 = v_1/norm(v_1);
    v_2 = obser(:,end);		v_2 = v_2/norm(v_2);
    A = [v_1 -C12*v_2];    	b = t12_1;
    lambda = A\b;
    f_c1 = lambda(1)*v_1;
end