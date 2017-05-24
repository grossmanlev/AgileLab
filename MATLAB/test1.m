ncomp = 2;

opts = struct( 'maxiters', 30,...
               'algorithm', 'vb',...
               'minangle', 0 );
           
[ A, S, Mu, V, cv, hp, lc ] = pca_full( W', ncomp, opts );