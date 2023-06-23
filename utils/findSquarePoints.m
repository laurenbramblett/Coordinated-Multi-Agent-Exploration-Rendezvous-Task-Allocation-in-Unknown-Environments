function sq_coords = findSquarePoints(x,y,radius)
    xcells = [x-radius,x+radius];
    ycells = [y-radius,y+radius];
    sq_coords = [xcells(1),ycells(1);xcells(2),ycells(1);xcells(1),ycells(2);xcells(2),ycells(2);
        xcells(1),y; xcells(2),y; x,ycells(1);x,ycells(2);x,y];
%     [m,n] = ndgrid(xcells,ycells);    
%     sq_coords = [m(:),n(:)];
end