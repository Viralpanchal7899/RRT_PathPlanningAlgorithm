function [x_i,y_i] = check_obstacle(i,path,obstacles,num_obstacles)

cols_x = [1,3,5,7];
rows_y = [2,4,6,8];

for j = 1:num_obstacles
    for c = 1:4
        for u = 1:num_obstacles
            x_i = 0;
            if path(i,2) >= obstacles(u,2) && path(i,2)<=obstacles(u,6)
            r_x = path(i,1) - obstacles(j,cols_x(c));
                if abs(r_x) <= 5
                    x_i = 1;
                end
            end
        end
        for u = 1:num_obstacles
            y_i = 0;
            if path(i,1)>=obstacles(u,1) && path(i,1)<=obstacles(u,3)
            r_y = path(i,2) - obstacles(j,rows_y(c));
                if abs(r_y) <=5
                    y_i = 1;
                end
            end
        end
    end
end
end

