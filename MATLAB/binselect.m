function V = binselect (vox)

[n, m] = size(vox);
V = speye(n);

V = V(~isnan(vox), :);
