% loadObjects loads the .txt files into either and train or test folder from path
%

% className is the name of the object clas (i.e. 'bottles')

% size is the 3D length (i.e. 30 x 30 x 30 voxel size is 30

% type is either 'test' or 'train'

function[W] = loadObjects(path, className, size, type)
% For now, assume he voxels will be stored in 100x100x100
is_occ = false;
og_path = pwd;

if ~(strcmp('test', type) || strcmp('train', type))
    disp('Invalid type!');
    return;
end

savePath = [pwd, '/Objects/', className, '/', int2str(size), '/', type, '/'];

cd(path);
F = dir ([path,'/*.txt']);

for f = 1:numel(F)
    %Open file
    fileID = fopen(F(f).name, 'r');
    
    %Take in input data
    sizeA = [3 Inf];
    A = fscanf(fileID, '%d %d %d', sizeA);
    A = A';
    % [m,n] = size(A);
    Vdata = A;

    %Create 3D voxel object matrix
    instance = zeros([size size size]);
    for i = Vdata'
        tmp = i';
        if(is_occ && tmp(1) > 0)
            instance(tmp(1)+50,tmp(2)+50,tmp(3)+50) = NaN; % or NaN
        else
            instance(tmp(1)+50,tmp(2)+50,tmp(3)+50) = 1; 
        end
    end;
    
    save([savePath, className, '_', int2str(f)],'instance');
    
end
cd(og_path);
end