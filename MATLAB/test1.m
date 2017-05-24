ncomp = 5;

opts = struct( 'maxiters', 5,...
               'algorithm', 'vb',...
               'minangle', 0 );
           
[ A, S, Mu, V, cv, hp, lc ] = pca_full( data', ncomp, opts );


% get the data in a 100000 x 5 matrix
% then data (data == 0) = NaN;
% feed data' to the pca_full, with ncomp 5, and then S' is the basis!