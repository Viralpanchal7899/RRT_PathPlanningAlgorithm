
clear all; close all;
figure(1); hold on;
axis([0 100 0 100]);
box on;

% Define and plot the start state for the planning problem
%               x  y
start_state = [ 5 50];
plot(start_state(1),start_state(2),'.r','MarkerSize',20);

% Define and plot the goal region for the planning problem
%              x1 y1  x2  y2   x3  y3  x4  y4
goal_region = [90  0 100   0  100 100  90 100];
goal_x = [goal_region(1) goal_region(3) goal_region(5) goal_region(7)];
goal_y = [goal_region(2) goal_region(4) goal_region(6) goal_region(8)];
patch(goal_x,goal_y,'green');

% Define and plot the locations of the obstacles
%             x1 y2 x2 y2 x3 y3 x4 y4
obstacles = [  5 10 15 10 15 20  5 20; % obstacle 1
              10 40 20 40 20 50 10 50; % obstacle 2
              20 70 30 70 30 80 20 80; % ...etc...
              30 20 40 20 40 30 30 30; 
              40 50 50 50 50 60 40 60;
              50  5 60  5 60 15 50 15;
              55 80 65 80 65 90 55 90;
              60 40 70 40 70 50 60 50;
              70 20 80 20 80 30 70 30
              75 65 85 65 85 75 75 75 ];

num_obstacles = size(obstacles,1);

for i_obs = 1:num_obstacles
    obs_x = [obstacles(i_obs,1) obstacles(i_obs,3) obstacles(i_obs,5) obstacles(i_obs,7)];
    obs_y = [obstacles(i_obs,2) obstacles(i_obs,4) obstacles(i_obs,6) obstacles(i_obs,8)];
    patch(obs_x,obs_y,'blue');
end

for z = 1:100
    [path,path_length] = RRT(start_state,obstacles);
    path = flipud(path);
    A = [1 0 0 0;
        0 1 0 0;
        1 0 1 0;
        0 1 0 1];
    
   p_k = eye(4);
   Q = eye(4);
   H_k = [1 0 0 0;
       0 1 0 0]; 
   for j = 1:size(path)
        [x_i, y_i] = check_obstacle(j,path,obstacles,num_obstacles);
        if x_i == 1
            H_k = [H_k;
                0 0 1 0];
        end
        if y_i == 1
            H_k = [H_k;
                0 0 0 1];
        end
        R_k = eye(size(H_k,1));
        p_k = ((A * p_k * A' + Q)^(-1) + (H_k' * R_k^(-1) * H_k))^(-1);
   end
   disp(z)
end
        
            

