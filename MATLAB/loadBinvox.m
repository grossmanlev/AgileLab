function [ OUT ] = loadBinvox( fileName, multiple )
%LOADBINVOX Summary of this function goes here
%   loads all binvox files (in .txt format) from a directory (multiple =
%   true)
%   or simply loads a single .txt file (if multiple = false)
index = 0;

if multiple
    files = dir([fileName, '/*.txt']);
    for file = 1:numel(files)
        disp(files(file).name);
        fileID = fopen([fileName, '/', files(file).name], 'r');
        formatSpec = '%d';
        sizeA = [30*30 30];
        A = fscanf(fileID, formatSpec, sizeA);
        instance = reshape(A, [30,30,30]);
        instance = permute(instance, [3 1 2]); % switch around the axes, so it's like binvox's specified... x, z, y
        %save(['/home/lev/AgileLab/MATLAB/BEOs/Objects/bottle/30/test/bottle_', num2str(index), '.mat'], 'instance');
        index = index + 1;
        OUT = instance;
    end
else
    file = fileName;
    fileID = fopen(file, 'r');
    formatSpec = '%d';
    sizeA = [30*30 30];
    A = fscanf(fileID, formatSpec, sizeA);
    instance = reshape(A, [30,30,30]);
    instance = permute(instance, [3 1 2]); % switch around the axes, so it's like binvox's specified... x, z, y
    instance = rotateObject(instance, affine3d, 3, 180);
    %save(['/home/lev/AgileLab/MATLAB/BEOs/Objects/bottle/30/test/bottle_', num2str(index), '.mat'], 'instance');
    OUT = instance;

end

end