load('tracks.mat');

pCi_list=[];
for kk=40:64
    Tcw=cameraParams.Tcb*Tbw(:,:,kk);
    
    pCi_list = [pCi_list, Tcw(1:3,1:3)*feat_position(:,481) + Tcw(1:3,4)];
end
a = pCi_list./pCi_list(3,:);
a = cameraParams.K*a;
a