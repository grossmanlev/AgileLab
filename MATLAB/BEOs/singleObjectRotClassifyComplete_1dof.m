 function [corseEDTRotationEstimate, fineRotationEstimate, estimatedClass, corseEDTReconstructedObject, fineReconstructedObject]...
        = singleObjectRotClassifyComplete_1dof(testObject, step, sharedMeans, sharedCovs, sharedBasisGPU, rotation)
    
    % we have a voxel object query consisting of 1s
    % for filled voxels, 0s for empty voxels, and -1s for unobserved voxes
    % this can be generated from a single depth image, or multiple depth images,
    % and is assumed to be aligned translationaly as well as rotationaly in the
    % x and y axis (up is up assumption). It is of unknown rotation about its Z-Axis and will
    % have its class, z-axis orientation, and unkown geometry jointly estimated.
    %
    % testObject is the voxel object (test object)
    % step is the distance (in degrees) between each pose estimation search point
    % sharedMeans are the component means (in shared object space) that where learned (one for each class)
    % sharedCovs are the covariences for each component (class) in the shared subspace
    % sharedBasisGPU is the subspace itself
    % rotation is a boolean flag, if true we estimate rotation, if false we assume object is already correctly rotated
    %
    %
    % Ignore the fact that some variables have 'GPU' in the name. An earlier
    % version of this had GPU accelleration but since adding ridge regression
    % that was reverted
    
    if nargin < 6
        rotation = true;
    end
    
    MSGID = 'MATLAB:nearlySingularMatrix';
    warning('off', MSGID);
    vobjectSize1 = size(testObject, 1);
    vobjectSize2 = size(testObject, 2);
    vobjectSize3 = size(testObject, 3);
    
    if rotation
        % first, estimate rotation with 1 degree precision using only reconstruction
        % error
        numSteps = numel(0:step:359);
        rots = cell(numSteps, 1); % hold rotations being considered
        % generate canidates, the correct one is the one rotated by the inverse of
        % the true rotation (R above)
        canidateRotObjVectors = zeros((floor(359/step) + 1), numel(testObject));
        for index=1:numel(rots)
            k = (index-1)*step;
            %testRot = compose_rotation_d(0, 0, k);
            testRot = compose_rotation_d(k, 0, 0); %y-axis rotation
            rots{index} = testRot;
            canidateRotObj = rotateObject(testObject, affine3d(), testRot);
            canidateRotObjVector = reshape(canidateRotObj, numel(canidateRotObj), 1, 1);
            canidateRotObjVectors(index, :) = canidateRotObjVector;
        end
        %canidateRotObjVectorsGPU = gpuArray(canidateRotObjVectors);
        canidateRotObjVectorsGPU = canidateRotObjVectors;
        
        % this function name is a bit misleading, it returns both class densities
        % for each canidate point (vector) as well as the points themselves (the
        % input are objects in 3D voxel space). Here, we're just using the
        % projections, we don't (yet) care about the class densities
        [~, gpuProjObjVec] = getDensities(sharedBasisGPU, sharedMeans, sharedCovs, canidateRotObjVectorsGPU);
        
        % this operation reprojects (completes) each of the canidate objects by
        % projecting back into voxel space
        %reconObjs =  gather(gpuProjObjVec * sharedBasisGPU');
        reconObjs =  gpuProjObjVec * sharedBasisGPU';
        
        % now that we have reconstructions for 1 degree precision, we find the
        % best estimate of pose based only on Euclidian Distance Transform Errors
        % between each canidate rotated object and the corosponding reprojection
        %
        % Side note: be careful with interObjectDist, it expects only the first
        % object to have -1 values (i.e. be partially completed).
        rerrors = [];
        for i=1:size(reconObjs, 1)
            corseReconstructedObject = reshape(reconObjs(i, :), vobjectSize1, vobjectSize2, vobjectSize3);
            corseReconstructedObject(corseReconstructedObject < 0.5) = 0;
            corseReconstructedObject(corseReconstructedObject >= 0.5) = 1;
            rerrors(end+1) = interObjectDist(corseReconstructedObject, testObject); %#ok<AGROW>
        end
        [~, i] = min(rerrors); % find the index where the lowest error occured
        % rots{i} is where we search arround now, get the angle about z axis (x and y will be zero here anyway);
        rotationHolder = rots{i};
        rotationHolder(4, 4) = 1;
        rotationHolder = affine3d(rotationHolder);
        
        % once we figure out the rotation, we need to invert it (the goal is to
        % figure out what rotation was applied to the input, we have the rotation
        % required to warp the input back to canonical, so we invert)
        corseEDTRotationEstimate = invert(rotationHolder);
        corseEDTRotationEstimate = corseEDTRotationEstimate.T(1:3, 1:3); % go from affine matrix to 3x3 rotation
        
        %[~, ~, z] = decompose_rotation_d(rots{i});
        [z, ~, ~] = decompose_rotation_d(rots{i}); %x-axis rotation
        
        corseEDTReconstructedObject = reshape(reconObjs(i, :), vobjectSize1, vobjectSize2, vobjectSize3);
        corseEDTReconstructedObject(corseEDTReconstructedObject < 0.5) = 0;
        corseEDTReconstructedObject(corseEDTReconstructedObject >= 0.5) = 1;
        
        % now the corse estimate is done, we finetune now in 0.1 degree intervuls spaning -10 to 10 degrees around
        % the reconstructionError estimate
        %
        % TODO: Don't be lazy, precompute the size of some of these variables
        newStep = step/10;
        fineGrainRots = {};
        fineCanidateRotObjVectors = [];
        fineGrainRotAngles = [];
        for newZ = z-10:newStep:z+10
            %fineGrainRots{end+1} = compose_rotation_d(0, 0, newZ); %#ok<AGROW> % get rotation matrix corosponding to angle
            fineGrainRots{end+1} = compose_rotation_d(newZ, 0, 0); %#ok<AGROW> % get rotation matrix corosponding to angle
            canidateRotObj = rotateObject(testObject, affine3d(), fineGrainRots{end}); % rotate the input object
            canidateRotObjVector = reshape(canidateRotObj, numel(canidateRotObj), 1, 1); % shape into vector
            fineCanidateRotObjVectors(end+1, :) = canidateRotObjVector; %#ok<AGROW> % save for later
            fineGrainRotAngles(end+1) = newZ; %#ok<AGROW> % store the corosponding angle about z axis for easy reference later
        end
        
    else
        fineCanidateRotObjVectors = reshape(testObject, numel(testObject), 1, 1)';
        fineGrainRots = {compose_rotation_d(0, 0, 0)};
        corseEDTRotationEstimate = fineGrainRots{1};
    end
    
    % now that we've tried a much finer resolution of rotations about our corse
    % estimate, project each of them onto the shared basis
    %fineCanidateRotObjVectorsGPU = gpuArray(fineCanidateRotObjVectors);
    fineCanidateRotObjVectorsGPU = fineCanidateRotObjVectors;
    
    % Get the projected objects and the densities for each projection (for each
    % class). We'll actually use the densities this time.
    [fineGpuDensities, fineGpuProjObjVec] = getDensities(sharedBasisGPU, sharedMeans, sharedCovs, fineCanidateRotObjVectorsGPU);
    %fineDensities = gather(fineGpuDensities);
    fineDensities = fineGpuDensities;
    
    % reconstruct the objects
    %fineReconObjs =  gather(fineGpuProjObjVec * sharedBasisGPU');
    fineReconObjs =  fineGpuProjObjVec * sharedBasisGPU';
    
    % figure out where the most probable (highest density) class is
    [maxPerClass, bestPosePerClass] = max(fineDensities, [], 2);
    [~, estimatedClass] = max(maxPerClass); % this is the class where the largest density occurs
    
    I = bestPosePerClass(estimatedClass); % this is the pose index where the largest density occurs
    
    % figure out what rotation corosponds to the pose index (I)
    rotationHolder = fineGrainRots{I};
    rotationHolder(4, 4) = 1;
    rotationHolder = affine3d(rotationHolder);
    
    % once we figure out the rotation, we need to invert it (the goal is to
    % figure out what rotation was applied to the input, we have the rotation
    % required to warp the input back to canonical, so we invert)
    fineRotationEstimate = invert(rotationHolder);
    fineRotationEstimate = fineRotationEstimate.T(1:3, 1:3); % go from affine matrix to 3x3 rotation
    
    % now that we have our class estimate (in estimated class) and our rotation
    % estimate (in probEstimatedRot), all that's left to do is save the
    % corosponding reconstruction
    fineReconstructedObject = reshape(fineReconObjs(I, :), vobjectSize1, vobjectSize2, vobjectSize3);
    fineReconstructedObject(fineReconstructedObject < 0.5) = 0; % NOTE: changing this threshold down yields more complete objects, abliet at a coast? (LG)
    fineReconstructedObject(fineReconstructedObject >= 0.5) = 1;
    
    if ~rotation % if we're not estimating rotation there is no difference between these two things
        corseEDTReconstructedObject = fineReconstructedObject;
    end
    warning('on', MSGID);
end

function [regProjectedObVecs, projectedObVecs] = projectObjs(objVectors, basis, mu)
    % given some object vectors from one or more objects in voxel space, project
    % onto shared basis, basis
	% TODO: Figure out how to do ridge regression on GPU
    
    if nargin < 3
        mu = 0;
    end
    
    if min(min(min(objVectors))) < 0
        incomplete = true; % incomplete objects
    else
        incomplete = false;
    end
    
    if iscolumn(objVectors) % if single vector, ensure is a row vector
        objVectors = objVectors';
    end
    
    % project
    if incomplete
        projectedObVecs = zeros(size(objVectors, 1), size(basis, 2));
        regProjectedObVecs = projectedObVecs;
        for obVec=1:size(objVectors, 1)
            objectVector = objVectors(obVec, :);
            goodInds = objectVector ~= -1;
            objectVector = objectVector - mu;
            
            X = basis(goodInds, :);
            A = X' * X;
            b = objectVector(goodInds) * basis(goodInds, :);
            regProjectedObVecs(obVec, :) = lasso(A, b, 'Alpha', 1, 'lambda', 0.00001);
            projectedObVecs(obVec, :) = A\b';
        end
    else
        for i=1:size(objVectors, 1)
            objVectors(i, :) = objVectors(i, :) - mu;
        end
        projectedObVecs = (objVectors) * basis;
        regProjectedObVecs = projectedObVecs;
    end
end

function [densities, projectedObVecs] = getDensities(sharedBasis, means, covs, rotatedObject)
    [regProjectedObVecs, projectedObVecs] = projectObjs(rotatedObject, sharedBasis);
    for classNum=1:numel(means)
        densities(classNum, :) = mvnpdf(regProjectedObVecs, means{classNum}, covs{classNum}); %#ok<AGROW>
    end
end
