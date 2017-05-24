function [O_rec] = reconstruct(W, O_p)
    V = binselect(O_p);
    A = (W') * (V') * V * W;
    b = (W') * (V') * (V * O_p);
    O_rec = W * (A \ b);
end

function V = binselect (vox)
    [n, ~] = size(vox);
    V = speye(n);
    V = V(~isnan(vox), :);
end




