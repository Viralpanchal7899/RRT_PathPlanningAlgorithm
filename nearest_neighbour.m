% Nearest neighbour 
% Viral Panchal - SIMLAB 2
% This function finds a nearest neigbour for the random point generated
% w.r.t the previously saved nodes in the tree_points matrix. 
% Output nn_index is the index of the node in the tree_point which is
% nearest to the random point.

function nn_index = nearest_neighbour(tree_points,px2,py2)
rand_new_pose = [px2 py2];
for i = 1: size(tree_points,1)
    d(i,1) = norm(rand_new_pose - tree_points(i,:));
end
[M,I] = min(d);
nn_index = I;
end