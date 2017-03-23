fileID = fopen('~/test/build/voxel.txt', 'r');
sizeA = [3 Inf];
A = fscanf(fileID, '%d %d %d', sizeA);
A = A';
[m,n] = size(A);

tID = fopen('~/test/build/transformation.txt', 'r');
T = fscanf(tID, '%d %d %d');
T = T';
T = -T;
T = T + 1;
T = repmat(T,m,1);

Vdata = A+T;

V = zeros(max(Vdata));

for i = Vdata'
    tmp = i';
    V(tmp(1),tmp(2),tmp(3)) = 1;
end;
s = size(V);
O = reshape(V,numel(V),1);

%data = O;

% [W, Y] = pca(data, 'VariableWeights', 'variance', 'Centered', true);
% W = diag(std(data))\W;
% 
% %# Getting mean and weights of data (for future data)
% [~, mu, we] = zscore(data);
% we(we==0) = 1;
% 
% %# New point in original 9dim system
% %# For example, it is the first point of our input data
% x = data(1,:);
% x = bsxfun(@minus,x, mu);
% x = bsxfun(@rdivide, x, we);
% 
% %# New coordinates as principal components
% y = x*W;
% y0 = Y(1,:);
% sum(abs(y0 - y)) %# 4.1883e-14 ~= 0