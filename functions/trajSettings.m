%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% <trajSettings.m specification>
% 1. Organizing the parameters for each trajectory 
% 2. exampleDataX.mat file is located in ./trajectories
% 3. featGenParams and distRange are set considering 
%     trajectory scale, body speed, density of features, etc.
% 
% INPUT   : selected trajectory type (during 'traj1', 'traj2', 'traj3')
% OUTPUT  : structure with trajtype, fileName, featGenParams, distRange
% FUNCTION: setting the trajectory options as a function form 

% 
% Tuning Parameter: featGenParams, distRange
% 
% Copyright (c) 2011 JaeYoung Chung (robot0321@github) All Rights Reserved
% Lisence: GNU GENERAL PUBLIC LICENSE Version 3, 29 June 2007
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function pathParams = trajSettings(trajtype)
    switch(trajtype)
        case 'traj1'
            fileName = 'exampleData1.mat';
            featGenParams = struct('Nfeatures',700,'step',5,'xyRange',100,'zRange',30);
            distRange = struct('mindist',2, 'maxdist',40);
        case 'traj2'
            fileName = 'exampleData2.mat';
            featGenParams = struct('Nfeatures',1000,'step',5,'xyRange',150,'zRange',60);
            distRange = struct('mindist',3, 'maxdist',150);
        case 'traj3'
            fileName = 'exampleData3.mat';
            featGenParams = struct('Nfeatures',500,'step',5,'xyRange',50,'zRange',20);
            distRange = struct('mindist',1, 'maxdist',60);
        otherwise
            error('not a proper option!!!')
    end
    pathParams = struct('pathName',trajtype,'fileName',fileName,'featGenParams',featGenParams,'distRange',distRange);
end