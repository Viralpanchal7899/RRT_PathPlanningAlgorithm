% Get Parent
% Viral Panchal - SIMLAB 2
% In this function we find the parent node index of a tree point to
% generate the final path from start to the goal region.

function new_parent_index = get_parent(parent_x, parent_y, tree_points)

for i = 1:size(tree_points,1)
    if parent_x == tree_points(i,1) && parent_y == tree_points(i,2)
        new_parent_index = i;
    end
end
