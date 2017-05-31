function [] = to_v(data)

%data = reshape(data, [100, 100, 100]); %resize to 100,100,100 matrix

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

vs = [0.5, 0.5, 0.5];
voxel_image(pts, vs);
view([-37.5, 30]);



