function[path,path_length] = RRT(start_state,obstacles)
e = 2;
Px_max = 100;
Py_max = 100;

px1 = start_state(1,1);
py1 = start_state(1,2);
tree_points = zeros();
tree_points(1,1) = px1(1,1);
tree_points(1,2) = py1(1,1);
parent_new(1,1) = zeros();
parent_new(1,2) = zeros();

rand_mat = zeros();
i = 1;
collision_stat = zeros;
j=1;
while tree_points(i,1)<90
rand_mat(i,1) = round(rand(1) * Px_max); 
rand_mat(i,2) = round(rand(1) * Py_max);
px2 = rand_mat(i,1);
py2 = rand_mat(i,2);
nn_index = nearest_neighbour(tree_points,px2,py2);
if nn_index == i
px1 = tree_points(nn_index,1);
py1 = tree_points(nn_index,2);
is_colliding = collision_check_segment(px1,py1,px2,py2,obstacles);
collision_stat(j,1) = is_colliding;
if  is_colliding == 0
    parent_new(i,1) = px1;
    parent_new(i,2) = py1;
    slope_m = (py2 - py1)/(px2 - px1);
    theta = atand(slope_m);
    i = i + 1;
    tree_points(i,1) = px1 + e*cosd(theta);
    tree_points(i,2) = py1 + e*sind(theta);
end
end
j=j+1;
end
hold on
path = tree_points;
path_length = 2 * size(path);
plot(path(:,1),path(:,2),'black');
end