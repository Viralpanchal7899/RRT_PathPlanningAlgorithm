% Sensor Check Obstacle
% Viral Panchal - SIMLAB 2
% In this function we check whether any obstacle is in range of the robot
% for every point of the path from start to goal region. 
% The functions considers obstacle sensing in both x(horizontal) and y
%(vertical) direction.

function [x_i,y_i] = sensor_check_obstacle(p_x,p_y,obstacles)

num_obstacles = size(obstacles,1);

for i_obs = 1:num_obstacles
    
    x1 = obstacles(i_obs,1); y1 = obstacles(i_obs,2);
    x2 = obstacles(i_obs,3); y2 = obstacles(i_obs,4);
    x3 = obstacles(i_obs,5); y3 = obstacles(i_obs,6);
    x4 = obstacles(i_obs,7); y4 = obstacles(i_obs,8);
    
    if (p_y >= y1) && (p_y <= y4)
        if abs(p_x - x1) <= 5 || abs(p_x - x2) <=5
            x_i = 1;
        else 
            x_i = 0;
        end
    else
        x_i = 0;
    end
    if (p_x >= x1) && (p_x <= x2)
        if abs(p_y - y1) <= 5 || abs(p_y - y3) <=5
            y_i = 1;
        else
            y_i = 0;
        end
    else
        y_i = 0;
    end        
end