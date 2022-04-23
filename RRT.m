function[path,path_length] = RRT(start_state,obstacles)
e = 2;
Px_max = 100;
Py_max = 100;
px1 = start_state(1,1);
py1 = start_state(1,2);
tree_points = zeros();
tree_points(1,1) = px1;
tree_points(1,2) = py1;
parent_new(1,1) = zeros();
parent_new(1,2) = zeros();
rand_mat = zeros();
i = 1;
while tree_points(i,1)<90
rand_mat(i,1) = rand(1) * Px_max; 
rand_mat(i,2) = rand(1) * Py_max;
px2 = rand_mat(i,1);
py2 = rand_mat(i,2); 
nn_index = nearest_neighbour(tree_points,px2,py2);
px1 = tree_points(nn_index,1);
py1 = tree_points(nn_index,2);
is_colliding = collision_check_segment(px1,py1,px2,py2,obstacles);
if  is_colliding == 0
    slope_m = (py2 - py1)/(px2 - px1);
    theta = atand(slope_m);
    i = i + 1;
    parent_new(i,1) = px1;
    parent_new(i,2) = py1;
    tree_points(i,1) = (px1 + e*cosd(theta));
    tree_points(i,2) = (py1 + e*sind(theta));
end
end
path = zeros();
m = 1;
path(m,1) = tree_points(i,1);
path(m,2) = tree_points(i,2);
while path(m,1)~= start_state(1,1) && path(m,2) ~= start_state(1,2)
    parent_x = path(m,1);
    parent_y = path(m,2);
    new_parent_index = get_parent(parent_x,parent_y,tree_points);
    m = m + 1;  
    path(m,1) = parent_new(new_parent_index,1);
    path(m,2) = parent_new(new_parent_index,2);
end

path_length = 2 * size(path,1);
% hold on;
% plot(path(:,1),path(:,2),'black','LineWidth',2);
% plot(path(1,1),path(1,2),'*');
end