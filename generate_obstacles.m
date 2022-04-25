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

f=1;
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
%         [x_i, y_i] = check_obstacle(j,path,obstacles,num_obstacles);
        p_x = path(j,1);
        p_y = path(j,2);
        [x_i, y_i] = sensor_check_obstacle(p_x,p_y,obstacles);
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
        std_x_y(j,1) = sqrt(p_k(3,3));
        std_x_y(j,2) = sqrt(p_k(4,4));
   end
   disp(z)
   global_std_x_y{z} = std_x_y;
   p_k_final(z,1) = trace(p_k);
   global_path{z} = path;
   global_path_length(z,1) = path_length;
end

[M_SP,I_SP] = min(global_path_length);
shortest_path = global_path{I_SP};
ra_rb = global_std_x_y{I_SP};
hold on
plot(shortest_path(:,1),shortest_path(:,2),'black','LineWidth',2);
plot(shortest_path(size(shortest_path,1),1),shortest_path(size(shortest_path,1),2),'*');
for e_sp = 1:size(shortest_path,1)
    ra = ra_rb(e_sp,1);
    rb = ra_rb(e_sp,2);
    x0 = shortest_path(e_sp,1);
    y0 = shortest_path(e_sp,2);
    h(e_sp,1) = ellipse(ra,rb,0,x0,y0,'r');
end
title('Shortest Path');

figure
plot(start_state(1),start_state(2),'.r','MarkerSize',20);
patch(goal_x,goal_y,'green');
for i_obs = 1:num_obstacles
    obs_x = [obstacles(i_obs,1) obstacles(i_obs,3) obstacles(i_obs,5) obstacles(i_obs,7)];
    obs_y = [obstacles(i_obs,2) obstacles(i_obs,4) obstacles(i_obs,6) obstacles(i_obs,8)];
    patch(obs_x,obs_y,'blue');
end

[min_uncern,I_min_uncern] = min(p_k_final);
min_uncern_path = global_path{I_min_uncern};
ra_rb_min_uncern = global_std_x_y{I_min_uncern};
hold on 
plot(min_uncern_path(:,1),min_uncern_path(:,2),'black','LineWidth',2);
plot(min_uncern_path(size(min_uncern_path,1),1),min_uncern_path(size(min_uncern_path,1),2),'*');
for e_sp = 1:size(min_uncern_path,1)
    ra = ra_rb_min_uncern(e_sp,1);
    rb = ra_rb_min_uncern(e_sp,2);
    x0 = min_uncern_path(e_sp,1);
    y0 = min_uncern_path(e_sp,2);
    h(e_sp,1) = ellipse(ra,rb,0,x0,y0,'r');
end
title('Min Uncerntainty path')

figure
plot(start_state(1),start_state(2),'.r','MarkerSize',20);
patch(goal_x,goal_y,'green');
for i_obs = 1:num_obstacles
    obs_x = [obstacles(i_obs,1) obstacles(i_obs,3) obstacles(i_obs,5) obstacles(i_obs,7)];
    obs_y = [obstacles(i_obs,2) obstacles(i_obs,4) obstacles(i_obs,6) obstacles(i_obs,8)];
    patch(obs_x,obs_y,'blue');
end
[max_uncern,I_max_uncern] = max(p_k_final);
max_uncern_path = global_path{I_max_uncern};
ra_rb_max_uncern = global_std_x_y{I_max_uncern};
hold on 
plot(max_uncern_path(:,1),max_uncern_path(:,2),'black','LineWidth',2);
plot(max_uncern_path(size(max_uncern_path,1),1),max_uncern_path(size(max_uncern_path,1),2),'*');
for e_sp = 1:size(max_uncern_path,1)
    ra = ra_rb_max_uncern(e_sp,1);
    rb = ra_rb_max_uncern(e_sp,2);
    x0 = max_uncern_path(e_sp,1);
    y0 = max_uncern_path(e_sp,2);
    h(e_sp,1) = ellipse(ra,rb,0,x0,y0,'r');
end
title('Max Uncerntainty path')
            

