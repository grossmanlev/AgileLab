% Lev Grossman: 7/19/17
% Script to continuously voxelize and reconstruct single binvox objects
% TODO: Add in a low-pass filter to "smooth" the newest voxel object
% against the previous ones
i = 0;
meshPath = '/home/lev/projects/tracking/src/dbot_getting_started/object_meshes/object_models/';
delete([meshPath, 'recon_*']);
alpha = 0.5; %dt/(RC+dt) constant
lastY =  zeros(30, 30, 30); % y[i-1] in low-pass algorithm

while true
    instance = loadBinvox('~/projects/tracking/voxels.txt', false);
    %instance(:,26:30,:) = 0; %HACKS
    %instance(:, :, 18:30) = 0; %HACKS
    %instance = imtranslate(instance, [0 0 2]);
    instance = rotateObject(instance, affine3d, 3, 180);
    instance(:,1:2,:) = 0;
    save('/home/lev/AgileLab/MATLAB/BEOs/Objects/bottle/30/test/bottle_2.mat', 'instance');

    objects = one_dof_full_BEO_pipeline('full', false, true);
    %visualizeObject(objects{1});
    objects{1} = rotateObject(objects{1}, affine3d, 3, 180); %rotate around z-axis
    
    % Low-Pass Filter Algorithm
    if i == 0
        lastY = alpha * objects{1};
    else
        objects{1} = (alpha * objects{1}) + ((1 - alpha) * lastY);
        lastY = objects{1};
    end
        
    thresh = mean(prctile(prctile(objects{1},90),90));
    objects{1}(objects{1} < thresh) = 0;
    objects{1}(objects{1} >= thresh) = 1;    
    
    gridCoffs = zeros(1, 30);
    for index = 1:30
        gridCoffs(index) = -0.5 + ((index - 1) * (1 / 29));
    end
    CONVERT_voxels_to_stl('tmp.stl', objects{1}, gridCoffs, gridCoffs, gridCoffs, 'binary');
    movefile('tmp.stl', [meshPath, 'recon_', num2str(i), '.stl']);
    i = i + 1;
    if(i >= 10)
        delete([meshPath, 'recon_', num2str(i-10), '.stl']);
    end
end