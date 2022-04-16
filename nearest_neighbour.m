% Nearest neighbour 

function nn_index = nearest_neighbour(tree_points,px2,py2)
rand_new_pose = [px2 py2];
for i = 1: size(tree_points)
    d(i,1) = norm(rand_new_pose - tree_points(i,:));
end
[M,I] = min(d);
nn_index = I;
end