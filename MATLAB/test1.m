ncomp = 5;

opts = struct( 'maxiters', 5,...
               'algorithm', 'vb',...
               'minangle', 0 );
           
[ A, S, Mu, V, cv, hp, lc ] = pca_full( W', ncomp, opts );