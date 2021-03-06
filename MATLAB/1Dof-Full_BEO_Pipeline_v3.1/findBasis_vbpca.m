function findBasis_vbpca
    % currently hardcoded for modelnet 10
    % run from the dir where this file is located
    
	% this can take a LOT of memory, if there are issues, try setting initalNumBasis to 50 and setting autoBasisSizeFlag to false.
    resolution = 30;
	autoBasisSizeFlag = true;
    initalNumBasis = 150;
	
	
    savePrefix = [pwd, 'ModelNet10_VBPCA_Learned_Subspaces/modelnet10_'];
    
    names = {'bathtub', 'bed', 'chair', 'desk', 'dresser', 'monitor', 'night_stand', 'sofa', 'table', 'toilet'};
    pathPrefix = [pwd, '/ModelNet10_Dataset/ModelNet10/Voxelized/'];
    pathPostfix = ['/', num2str(resolution), '/train'];
    VBPCA_calculation(resolution, initalNumBasis, savePrefix, names, pathPrefix, pathPostfix); % find basis
	if autoBasisSizeFlag
		autoBasisSize(resolution, initalNumBasis, savePrefix, names); % auto basis size
	end
end
%------------------- end hardcoded stuff --------------------- %
function VBPCA_calculation(resolution, initalNumBasis, savePrefix, names, pathPrefix, pathPostfix)
    
    for i=1:numel(names)
        name = names{i};
        disp(['Processing ', name, ' class']);
        path = [pathPrefix, name, pathPostfix];
        transformedObjects = loadModelNetObjects(path);
        
        % convert each object into a column vector and then combine into data matrix
        dataMatrix = reshape(transformedObjects,...
            size(transformedObjects, 1)*size(transformedObjects, 2)*size(transformedObjects, 3),...
            size(transformedObjects, 4));
        clear('objects', 'transVecs', 'rtforms', 'transformedObjects');
        disp('Starting VBPCAd');
        [A, S, Mu, V, CV, HP, LC] = pca_diag(dataMatrix, initalNumBasis, 'savebest', true, 'maxiters', 10000); %#ok<ASGLU>
        save([savePrefix, name, '_', num2str(initalNumBasis), '_basis_size_', num2str(resolution), '_vobject.mat'], '-v7.3')
    end
end


function autoBasisSize(resolution, initalNumBasis, savePrefix, names)
    varthresh = 0.6;
    
    post = ['_', num2str(initalNumBasis), '_basis_size_', num2str(resolution), '_vobject.mat'];
    savePost = ['_auto_basis_size_', num2str(resolution), '_vobject.mat'];
    
    % load trained subspaces
    numClasses = numel(names);
    subspaces = cell(numClasses, 1);
    Ws = cell(numClasses, 1);
    autoWs = cell(numClasses, 1);
    eigenValues = cell(numClasses, 1);
    propVarience = cell(numClasses, 1);
    cumPropVarience = cell(numClasses, 1);
    for j=1: numClasses
        subspaces{j} = load([savePrefix, names{j}, post]);
        Ws{j} = subspaces{j}.A;
        eigenValues{j} = (sqrt(sum((Ws{j}).^2)))';
        [eigenValues{j}, IX] = sort(eigenValues{j}, 'descend');
        Ws{j} = Ws{j}(:, IX);
        
        totalVarience = sum(eigenValues{j});
        propVarience{j} = eigenValues{j} / totalVarience;
        cumPropVarience{j} = cumsum(propVarience{j});
        mask = cumPropVarience{j} <= varthresh;
        autoWs{j} = Ws{j}(:, mask);
        subspaces{j}.A = autoWs{j};
        save([savePrefix, names{j}, savePost], '-struct', 'subspace');
    end
end

