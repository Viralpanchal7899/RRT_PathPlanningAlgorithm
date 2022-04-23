function new_parent_index = get_parent(parent_x, parent_y, tree_points)

for i = 1:size(tree_points,1)
    if parent_x == tree_points(i,1) && parent_y == tree_points(i,2)
        new_parent_index = i;
    end
end
