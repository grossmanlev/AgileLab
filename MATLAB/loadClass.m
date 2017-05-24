% loadClass loads the .txt files into a matrix from path
%
% is_occ   - true  : partial object creation (test)
%          - false : normal
%
% is_basis - true  : loading a full object class
%          - false : loading object

function[W] = loadClass(path, is_occ, is_basis)
original_dir = pwd;
% For now, assume he voxels will be stored in 100x100x100


cd (path);
F = dir ('*.txt');

for f = 1:size(F,1)
    %Open file
    fileID = fopen(F(f).name, 'r');
    
    %Take in input data
    sizeA = [3 Inf];
    A = fscanf(fileID, '%d %d %d', sizeA);
    A = A';
    % [m,n] = size(A);
    Vdata = A;

    %Create 3D voxel object matrix
    V = zeros([100 100 100]);
    for i = Vdata'
        tmp = i';
        if(is_occ && tmp(1) > 0)
            V(tmp(1)+50,tmp(2)+50,tmp(3)+50) = NaN; % or NaN
        else
            V(tmp(1)+50,tmp(2)+50,tmp(3)+50) = 1; 
        end
    end;
    
    %Add voxelized object vectors into W
    if f == 1
        W = reshape(V,numel(V),1);
    else
        W = [W, (reshape(V,numel(V),1))];
    end
end

if(is_basis)
    [W, ~, ~] = svd(W, 'econ');
end

cd (original_dir);
end
