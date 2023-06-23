function [obs, map, x_scale, y_scale] = setMap(whichMap)
x_scale = 0; y_scale = 0;
switch whichMap
    case "mapY"
        M0 = ones(101)*0.5;
        obs_r = [40:60; 40:60]';
        obs_l = [40:60; 30:-1:10]';
        obs_r3 = [(40:60)-1; 40:60]';
        obs_l3 = [(40:60)-1; 30:-1:10]';
        obs_r2 = [(40:60)+10; 35:55]';
        obs_l2 = [(40:60)+10; 35:-1:15]';
        obs_r4 = [(40:60)+9; 35:55]';
        obs_l4 = [(40:60)+9; 35:-1:15]';
        obs_bl = [30:40; ones(1,11)*30]';
        obs_br = [30:40; ones(1,11)*40]';
        obs_b = [30*ones(1,11); 30:40]';
        obs = [obs_r; obs_l; obs_r2; obs_l2; obs_r3; obs_l3;obs_r4; obs_l4;obs_bl; obs_br; obs_b];
        obs = [obs(:,2) obs(:,1)];
        idx = sub2ind(size(M0),obs(:,1),obs(:,2));
        M0(idx) = 1;
        map = M0;
        %contour(M0)
        % MO = (M0');
        % contour(MO)
        % map = binaryOccupancyMap(MO>0.85);
        % figure()
        % show(map)
        % contour(M0); colormap(jet); shading interp; axis square;
    case "mapX"
        M0 = zeros(20);
        slant_side1 = [repelem(10,length(6:13));6:13]';
        slant_side2 = [repelem(11,length(6:13));6:13]';
        slant_up1 = [6:13;repelem(10,length(6:13))]'; 
        slant_up2 = [6:13;repelem(11,length(6:13))]';
        obs = [slant_side1; slant_side2; slant_up1; slant_up2];
        obs = [obs(:,2) obs(:,1)];
        idx = sub2ind(size(M0),obs(:,1),obs(:,2));
        M0(idx) = 1;
        map = M0;
%         pcolor(map); colormap(flipud(bone))
        
%%%%%%%%%%%%%%%%%MAP PLAYGROUND%%%%%%%%%%%%%%%%%%%%%
    case "playground"
        image = imread('imageMap.png');
        grayimage = rgb2gray(image);
        bwimage = grayimage < 0.5;
        grid = binaryOccupancyMap(bwimage);
%         show(grid)
        M0 = flip(occupancyMatrix(grid),1);
%         f = figure();
%         contour(M0)
        [row,col] = find(M0 == 1);
        obs = [col,row];
        tmp = zeros(length(M0));
        idx = sub2ind(size(tmp),obs(:,1),obs(:,2));
        tmp(idx) = 1;
        map = tmp;
%         contour(tmp)
%         f = figure();
%         show(binaryOccupancyMap(tmp'))
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%MAP MAZE%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    case "maze"
        map = mapMaze(10,2,'MapSize',[75 75],'MapResolution',1);
        grid = binaryOccupancyMap(map);
        % figure();
        % show(grid)
        M0 = flip(occupancyMatrix(grid),1);
        [row,col] = find(M0 == 1);
        obs = [col,row];
        tmp = zeros(length(M0));
        idx = sub2ind(size(tmp),obs(:,1),obs(:,2));
        tmp(idx) = 1;
        map = tmp;
%         f = figure();
%         show(binaryOccupancyMap(tmp'))

    case "complex"
        load("exampleMaps.mat")
        grid = binaryOccupancyMap(complexMap);
        % figure();
        % show(grid)
        M0 = flip(occupancyMatrix(grid),1);
        [row,col] = find(M0 == 1);
        obs = [col,row];
        tmp = zeros(length(M0));
        idx = sub2ind(size(tmp),obs(:,1),obs(:,2));
        tmp(idx) = 1;
        map = tmp;
%         show(binaryOccupancyMap(tmp'))
    case "warehouse"
        load("warehouseMaps.mat")
        grid = binaryOccupancyMap(logicalMap);
%         figure();
%         show(grid)
        M0 = flip(occupancyMatrix(grid),1);
        M0 = occupancyMatrix(grid);
        [row,col] = find(M0 == 1);
        obs = [col,row];
        tmp = zeros(length(M0));
        idx = sub2ind(size(tmp),obs(:,1),obs(:,2));
        tmp(idx) = 1;
        map = tmp;
%         f = figure();
%         show(binaryOccupancyMap(tmp'))
    case "clutter"
        map = mapClutter(15,{'Box','Circle','Plus'},'MapSize',[200 200],'MapResolution',1);
        grid = binaryOccupancyMap(map);
        % figure();
%         show(grid)
        M0 = flip(occupancyMatrix(grid),1);
        [row,col] = find(M0 == 1);
        obs = [col,row];
        tmp = zeros(length(M0));
        idx = sub2ind(size(tmp),obs(:,1),obs(:,2));
        tmp(idx) = 1;
        map = tmp;
        
    case "office_space"
        x_s = -22.8; y_s = -20.88; res = 0.02;
        gridSize = [50,50];
        image = imread('office_space.pgm');
        x_crop = [401:1500]; y_crop = [1:1100];
        imageCropped = image(y_crop,x_crop);
%         imshow(imageCropped)
        outputImage = imresize(imageCropped, gridSize);
%         imshow(outputImage)
        new_res = length(y_crop)*res/gridSize(1);
        new_origin = [x_s + (x_crop(1)-1)*res, y_s + (size(image,2)-y_crop(end))*res];
        x_scale = zeros(gridSize); y_scale = zeros(gridSize);
        
        for i = 1:gridSize(1)
           x_scale(:,i) = new_origin(1) + new_res*(i-1);
           y_scale(gridSize(1)-i+1,:) = new_origin(2) + new_res*(i-1);
        end
        
        imageNorm = double(outputImage)/255;
        imageOccupancy = 1-imageNorm;
        imageOccupancy = imageOccupancy>0.02;
        Gx = [1 0 -1; 2 0 -2; 1 0 -1];
        Gy = [1 2 1; 0 0 0; -1 -2 -1];

        G_Mx = filter2(Gx,imageOccupancy);
        G_My = filter2(Gy,imageOccupancy);
        S = sqrt(G_Mx.^2 + G_My.^2);
        tmp = S>1;
        
        grid = binaryOccupancyMap(tmp,1);
        M0 = flip(occupancyMatrix(grid),1);
%         added_obs = [39,13; 17, 19;...
%                     18,20; 19,19; 19,20;...
%                     16 16; 36 16; 36,15;
%                     16 20; 17 20; 34 13; 
%                     34 17; 36 17; 35 17;
%                     43 9; 41 9; 37 11; 
%                     36 11; 37 10; 41 9;
%                     41 8; 41 7; 40 8;
%                     40 7; 39 8; 42 11;
%                     40 6];
%         ind = sub2ind(size(M0),added_obs(:,1),added_obs(:,2));
%         M0(ind) = 1;
%         remove_obs = [39 15; 43 8; 44 9; 
%                      45 9; 44 10; 36 8;
%                      37 7; 38 6; 39 5;
%                      40 4; 41 5; 42 6; 
%                      43 7; 41 13];  
%         ind = sub2ind(size(M0),remove_obs(:,1),remove_obs(:,2));
%         M0(ind) = 0;
%         
        [row,col] = find(M0 == 1);
        obs = [col,row; 50 2];
        
        tmp = zeros(length(M0));
        idx = sub2ind(size(tmp),obs(:,1),obs(:,2));
        tmp(idx) = 1;
        map = tmp;
        Mtmp = M0;
        pcolor(Mtmp); colormap(flipud(bone))
        
    case "office_space_full"
        x_s = -22.8; y_s = -20.88; res = 0.02;
        image = imread('fullOfficeMap.pgm');
        image = [image(1:200,:); image];
%         imshow(image)
        gridSize = [200, 200];
        y_crop = 1:2200; x_crop = 451:2650;
        new_origin = [x_s + (x_crop(1)-1)*res, y_s + (size(image,2)-y_crop(end))*res];
        imageCropped = image(y_crop,x_crop);
%         imshow(imageCropped)
        outputImage = imresize(imageCropped, gridSize);
        new_res = length(y_crop)*res/gridSize(1);

        x_scale = zeros(gridSize); y_scale = zeros(gridSize);
        for i = 1:gridSize(1)
           x_scale(:,i) = new_origin(1) + new_res*(i-1);
           y_scale(gridSize(1)-i+1,:) = new_origin(2) + new_res*(i-1);
        end

        imageNorm = double(outputImage)/255;
        imageOccupancy = 1-imageNorm;
        imageOccupancy = imageOccupancy>0.07;
        Gx = [1 0 -1; 2 0 -2; 1 0 -1];
        Gy = [1 2 1; 0 0 0; -1 -2 -1];

        G_Mx = filter2(Gx,imageOccupancy);
        G_My = filter2(Gy,imageOccupancy);
        S = sqrt(G_Mx.^2 + G_My.^2);
        S([1 size(G_Mx,1)],:) = 0;
        S(:,[1 size(G_My,2)]) = 0;
        tmp = S>1;


        grid = binaryOccupancyMap(tmp,1);
%         show(grid)
        inflate(grid,1);
        M0 = flip(occupancyMatrix(grid),1);
        [row,col] = find(M0 == 1);
        obs = [col,row];

        tmp = zeros(size(M0,2),size(M0,1));
        idx = sub2ind(size(tmp),obs(:,1),obs(:,2));
        tmp(idx) = 1;
        map = tmp;
        Mtmp = M0;
%         pcolor(Mtmp); colormap(flipud(bone))
        
    case "bubbles"
        x_s = -20.24; y_s = -10; res = 0.02;
        image = imread('bubble_mapv2.pgm');
        %image = [image(1:200,:); image];
%         imshow(image)
        gridSize = [40, 40];
        y_crop = 537:1536; x_crop = 501:1500;
        new_origin = [x_s + (x_crop(1)-1)*res, y_s + (size(image,2)-y_crop(end))*res];
        imageCropped = image(y_crop,x_crop);
%         imshow(imageCropped)
        outputImage = imresize(imageCropped, gridSize);
        new_res = length(y_crop)*res/gridSize(1);

        x_scale = zeros(gridSize); y_scale = zeros(gridSize);
        for i = 1:gridSize(1)
           x_scale(:,i) = new_origin(1) + new_res*(i-1);
           y_scale(gridSize(1)-i+1,:) = new_origin(2) + new_res*(i-1);
        end

        imageNorm = double(outputImage)/255;
        imageOccupancy = 1-imageNorm;
        imageOccupancy = imageOccupancy>0.07;
        Gx = [1 0 -1; 2 0 -2; 1 0 -1];
        Gy = [1 2 1; 0 0 0; -1 -2 -1];

        G_Mx = filter2(Gx,imageOccupancy);
        G_My = filter2(Gy,imageOccupancy);
        S = sqrt(G_Mx.^2 + G_My.^2);
        S([1 size(G_Mx,1)],:) = 0;
        S(:,[1 size(G_My,2)]) = 0;
        tmp = S>1;


        grid = binaryOccupancyMap(tmp,1);
        inflate(grid,1)
%         show(grid)
        M0 = flip(occupancyMatrix(grid),1);
        [row,col] = find(M0 == 1);
        obs = [col,row];
        tmp = zeros(size(M0,2),size(M0,1));
        idx = sub2ind(size(tmp),obs(:,1),obs(:,2));
        tmp(idx) = 1;
        map = tmp;
        Mtmp = M0;
%         pcolor(Mtmp); colormap(flipud(bone))

    case "lab_setup1"
        x_s = -100; y_s = -100; res = 0.1;
        image = imread('secondmap.pgm');
        % imshow(image)

        more_crop_y = 955:1010; %perfect;
        more_crop_x = 980:1020;%perfect

        %make square
        square_y = 955:1010;
        square_x = 972:1027;
        x_crop = square_x; y_crop = square_y;
        imageCropped_square = image(square_y,square_x);
        % imshow(imageCropped_square)

        new_origin = [x_s + (x_crop(1)-1)*res, y_s + (y_crop(1)-1)*res];

        new_res = res;% new_res = length(y_crop)*res/gridSize(1);
        gridSize = size(imageCropped_square);
        x_scale = zeros(gridSize); y_scale = zeros(gridSize);
        for i = 1:gridSize(1)
           x_scale(:,i) = new_origin(1) + new_res*(i-1);
           y_scale(gridSize(1)-i+1,:) = new_origin(2) + new_res*(i-1);
        end

        imageNorm = double(imageCropped_square)/255;
        imageOccupancy = 1-imageNorm;
        imageOccupancy = imageOccupancy>0.07;
        Gx = [1 0 -1; 2 0 -2; 1 0 -1];
        Gy = [1 2 1; 0 0 0; -1 -2 -1];

        G_Mx = filter2(Gx,imageOccupancy);
        G_My = filter2(Gy,imageOccupancy);
        S = sqrt(G_Mx.^2 + G_My.^2);
        S([1 size(G_Mx,1)],:) = 0;
        S(:,[1 size(G_My,2)]) = 0;
        tmp = S>1;

        %bounding box
        tmp([54,3],:) = 1;
        tmp(:,[49,11]) = 1;
        grid = binaryOccupancyMap(tmp,1);

        M0 = flip(occupancyMatrix(grid),1);
        [row,col] = find(M0 == 1);
        obs = [col,row];
        tmp = zeros(size(M0,2),size(M0,1));
        idx = sub2ind(size(tmp),obs(:,1),obs(:,2));
        tmp(idx) = 1;
        map = tmp;
        Mtmp = M0;
        % pcolor(Mtmp); colormap(flipud(bone))
    case "lab_setup1_big"
        x_s = -100; y_s = -100; res = 0.1;
        image = imread('secondmap.pgm');
        % imshow(image)

        more_crop_y = 955:1010; %perfect;
        more_crop_x = 980:1020;%perfect

        %make square
        square_y = 955:1010;
        square_x = 972:1027;
        x_crop = square_x; y_crop = square_y;
        imageCropped_square = image(square_y,square_x);
        % imshow(imageCropped_square)
        resize = imresize(imageCropped_square,2/7);
        % imshow(resize)
        new_origin = [x_s + (x_crop(1)-1)*res, y_s + (y_crop(1)-1)*res];

        new_res = length(y_crop)*res/size(resize,1);
        gridSize = size(resize);
        x_scale = zeros(gridSize); y_scale = zeros(gridSize);
        for i = 1:gridSize(1)
           x_scale(:,i) = new_origin(1) + new_res*(i-1);
           y_scale(gridSize(1)-i+1,:) = new_origin(2) + new_res*(i-1);
        end

        imageNorm = double(resize)/255;
        imageOccupancy = 1-imageNorm;
        imageOccupancy = imageOccupancy>0.07;
        % Gx = [1 0 -1; 2 0 -2; 1 0 -1];
        % Gy = [1 2 1; 0 0 0; -1 -2 -1];
        % 
        % G_Mx = filter2(Gx,imageOccupancy);
        % G_My = filter2(Gy,imageOccupancy);
        % S = sqrt(G_Mx.^2 + G_My.^2);
        % S([1 size(G_Mx,1)],:) = 0;
        % S(:,[1 size(G_My,2)]) = 0;
        % tmp = S>1;
        tmp = imageOccupancy;
        %bounding box
        tmp([1,16],:) = 1;
        tmp(:,[3,16]) = 1;
        grid = binaryOccupancyMap(tmp,1);

        M0 = flip(occupancyMatrix(grid),1);
        [row,col] = find(M0 == 1);
        obs = [col,row];
        tmp = zeros(size(M0,2),size(M0,1));
        idx = sub2ind(size(tmp),obs(:,1),obs(:,2));
        tmp(idx) = 1;
        map = tmp;
        Mtmp = M0;
%         pcolor(Mtmp); colormap(flipud(bone))
    case "lab_setup_LShape_2Obs"
        load('labOccGrid_2Obs_L.mat','occGrid')
        orig_origin = [-1.942,2.443];
        new_origin = [-2.443, -2.443];
        sizeCoords = [abs(new_origin(1)*2), abs(new_origin(2)*2)];
        cells = 16;
        res = sizeCoords(1)/cells;

        gridSize = [cells,cells];
        x_scale = zeros(gridSize); y_scale = zeros(gridSize);
        for i = 1:gridSize(1)
           x_scale(:,i) = new_origin(1) + res*(i-1);
           y_scale(gridSize(1)-i+1,:) = new_origin(2) + res*(i-1);
        end
        
%         M0 = flip(occupancyMatrix(occGrid),1);
        M0 = occGrid;
        [row,col] = find(M0 == 1);
        obs = [col,row];
        tmp = zeros(size(M0,2),size(M0,1));
        idx = sub2ind(size(tmp),obs(:,1),obs(:,2));
        tmp(idx) = 1;
        map = tmp;
        Mtmp = M0;
        pcolor(Mtmp); colormap(flipud(bone))
    case "lab_setup_v2_2Obs"
        load('20Feb_labOccGrid_2Obs_L.mat','occGrid')
        orig_origin = [-1.942,2.443];
        new_origin = [-2.443, -2.443];
        sizeCoords = [abs(new_origin(1)*2), abs(new_origin(2)*2)];
        cells = 16;
        res = sizeCoords(1)/cells;

        gridSize = [cells,cells];
        x_scale = zeros(gridSize); y_scale = zeros(gridSize);
        for i = 1:gridSize(1)
           x_scale(:,i) = new_origin(1) + res*(i-1);
           y_scale(gridSize(1)-i+1,:) = new_origin(2) + res*(i-1);
        end
        
%         M0 = flip(occupancyMatrix(occGrid),1);
        M0 = occGrid;
        [row,col] = find(M0 == 1);
        obs = [col,row];
        obs = [obs; 6,7; 6,8; 9,3; 10,3]; %Add buffer
        tmp = zeros(size(M0,2),size(M0,1));
        idx = sub2ind(size(tmp),obs(:,1),obs(:,2));
        tmp(idx) = 1;
        map = tmp;
        Mtmp = M0;
%         pcolor(tmp); colormap(flipud(bone))
       
    case "lab_setup_v2_3Obs"
        load('20Feb_labOccGrid_3Obs_L.mat','occGrid')
        orig_origin = [-1.942,2.443];
        new_origin = [-2.443, -2.443];
        sizeCoords = [abs(new_origin(1)*2), abs(new_origin(2)*2)];
        cells = 16;
        res = sizeCoords(1)/cells;

        gridSize = [cells,cells];
        x_scale = zeros(gridSize); y_scale = zeros(gridSize);
        for i = 1:gridSize(1)
           x_scale(:,i) = new_origin(1) + res*(i-1);
           y_scale(gridSize(1)-i+1,:) = new_origin(2) + res*(i-1);
        end
        
%         M0 = flip(occupancyMatrix(occGrid),1);
        M0 = occGrid;
        [row,col] = find(M0 == 1);
        obs = [col,row];
        obs = [obs; 6,7; 6,8; 9,3; 10,3]; %Add buffer
        tmp = zeros(size(M0,2),size(M0,1));
        idx = sub2ind(size(tmp),obs(:,1),obs(:,2));
        tmp(idx) = 1;
        map = tmp;
        Mtmp = M0;
%         pcolor(tmp); colormap(flipud(bone))
    case 'simpleMap'
        load("exampleMaps.mat")
        grid = binaryOccupancyMap(simpleMap);
        % figure();
        show(grid)
        M0 = flip(occupancyMatrix(grid),1);
        [row,col] = find(M0 == 1);
        obs = [col,row];
        tmp = zeros(length(M0));
        idx = sub2ind(size(tmp),obs(:,1),obs(:,2));
        tmp(idx) = 1;
        map = tmp;
    case 'RahulsWorld'
        load('RahulsWorld.mat')
%         fullmap_rot = flipud(rot90(fullmap,-1));
       
        grid = binaryOccupancyMap(fullmap);
%         inflate(grid,1);
        figure();
        show(grid)
        occ = occupancyMatrix(grid);
        occ = [ones(30,1) occ ones(30,1)];
        occ = [occ; ones(1,32)];
        occMap = binaryOccupancyMap(occ);
        show(occMap)
        M0 = flip(occupancyMatrix(occMap),1);
        added_obs = [[3:12 3:12,repelem(25,length(3:9))]; ...
            [repelem(19,length(3:12)),repelem(18,length(3:12)),3:9]]';
        ind = sub2ind(size(M0),added_obs(:,1),added_obs(:,2));
        M0(ind) = 1;
        remove_obs = [repelem(22,length(3:9));3:9]';
        ind = sub2ind(size(M0),remove_obs(:,1),remove_obs(:,2));
        M0(ind) = 0;
        [row,col] = find(M0 == 1);
        obs = [col,row];
        
        tmp = zeros(length(M0));
        idx = sub2ind(size(tmp),obs(:,1),obs(:,2));
        tmp(idx) = 1;
        map = tmp;
        
        pcolor(tmp'); colormap(flipud(bone))
%         
end