function [vobj] = to_v(data)

[M,N,Z] = size(data);
vobj = [];

for i = 1:M
    for j = 1:N
        for k = 1:Z
            if data(i,j,k) > 0.1
                %fprintf('%d %d %d\n', i,j,k);
                tmp = [i j k];
                vobj = [vobj; tmp];
            end
        end
    end
end

