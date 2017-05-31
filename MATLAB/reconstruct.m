function [] = reconstruct(W, O_p)
    % show partial object
    vs = [1, 1, 1];
    voxel_image(to_v(O_p), vs);
    view([-37.5, 30]);
    
    clc;
    disp('Press any key to see reconstructed object')
    pause;
        
    % create reconstructed object
    V = binselect(O_p);
    A = (W') * (V') * V * W;
    b = (W') * (V') * (V * O_p);
    O_rec = W * (A \ b);
    
    % show reconstructed object
    voxel_image(to_v(O_rec), vs);
    view([-37.5, 30]);
    
    pause; clc;
    
    
end

function V = binselect (vox)
    [n, ~] = size(vox);
    V = speye(n);
    V = V(~isnan(vox), :);
end


function [pts] = to_v(data)

data = reshape(data, [100, 100, 100]); %resize to 100,100,100 matrix

[M,N,Z] = size(data);
pts = [];

for i = 1:M
    for j = 1:N
        for k = 1:Z
            if data(i,j,k) > 0.1
                %fprintf('%d %d %d\n', i,j,k);
                tmp = [i j k];
                pts = [pts; tmp];
            end
        end
    end
end

end



