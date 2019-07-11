function [Tbw, tw_wb] = getTrajectory(dataPath)
    load(dataPath, 'dataset');
    tw_wb = dataset.Pw_wb;
    if isfield(dataset, 'attitude')
        if isfield(dataset.attitude, 'quat'), Rwb = quat2rotm(dataset.attitude.quat);
        elseif isfield(dataset.attitude, 'dcm'), Rwb = dataset.attitude.dcm;
        elseif isfield(dataset.attitude, 'euler'), Rwb = eul2rotm(dataset.attitude.euler);
        else, error('no proper attitude option');
        end
    elseif isfield(dataset, 'heading')
        heading = pi/180*dataset.heading; % yaw (rad)
        pitch = zeros(size(heading));
        roll = pi*ones(size(heading));
        Rwb = angle2dcm(heading, pitch, roll); % dcm rotation matrix
    end
    tb_bw = zeros(3,1,size(tw_wb,2));
    for i=1:size(tw_wb,2)
        tb_bw(:,1,i) = -Rwb(:,:,i)'*tw_wb(:,i); 
    end
    Tbw = [permute(Rwb,[2,1,3]), tb_bw; zeros(1,3,length(tw_wb)),ones(1,1,length(tw_wb))]; % T^b_w
end