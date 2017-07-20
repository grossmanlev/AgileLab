% Lev Grossman: 7/19/17
% Script to continuously voxelize and reconstruct single binvox objects
i = 0;
while true
    instance = loadBinvox('~/projects/tracking/voxels.txt', false);
    %instance(:,26:30,:) = 0; %HACKS
    %instance(:, :, 18:30) = 0; %HACKS
    %instance = imtranslate(instance, [0 0 2]);
    save('/home/lev/AgileLab/MATLAB/BEOs/Objects/bottle/30/test/bottle_2.mat', 'instance');

    objects = one_dof_full_BEO_pipeline('full', false, true);
    %visualizeObject(objects{1});
    %objects{1} = rotateObject(objects{1}, affine3d, 3, 180); %rotate around z-axis

    gridCoffs = zeros(1, 30);
    for index = 1:30
        gridCoffs(index) = -0.5 + ((index - 1) * (1 / 29));
    end
    CONVERT_voxels_to_stl('tmp.stl', objects{1}, gridCoffs, gridCoffs, gridCoffs, 'binary');
    meshPath = '/home/lev/projects/tracking/src/dbot_getting_started/object_meshes/object_models/';
    movefile('tmp.stl', [meshPath, 'recon_', num2str(i), '.stl']);
    i = i + 1;
    if(i > 1)
        delete([meshPath, 'recon_', num2str(i-2), '.stl']);
    end
end