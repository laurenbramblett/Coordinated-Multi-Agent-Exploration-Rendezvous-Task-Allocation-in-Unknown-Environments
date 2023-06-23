function [x_grid,y_grid] = findGrid(x,y,x_sr,y_sr)
    binary_x = x_sr<x;
    x_grid = sum(binary_x);
    binary_y = y_sr<y;
    y_grid = sum(binary_y);
end