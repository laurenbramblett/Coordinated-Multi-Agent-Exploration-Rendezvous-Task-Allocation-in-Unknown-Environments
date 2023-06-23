function M_new = updateOccupancyMap(M0, GridPoints,x0,r,obs,cells)
% x0 = obj.x(k,:)-0.5;
        mu = r*3/4;
        IDx = rangesearch(GridPoints,x0,r,'Distance','euclidean');
        range = GridPoints(IDx{1},:);
        prob = ones(length(IDx{1}),1)*0.5;
        obsIDx = ismember(range,obs,'rows');
        obsInRange = range(obsIDx,:);
        for k = 1:length(range)
            pt = range(k,:);
            [xb, yb] = bresenham(x0(1),x0(2),pt(1),pt(2));
            tell(k) = any(ismember([xb, yb],obsInRange,'rows'));
            if length(xb)>1
                between(k) = any(ismember([xb(1:end-1), yb(1:end-1)],obsInRange,'rows'));
            else
                between(k) = 0;
            end
        end
        % Calculate the probability of being occupied
        d = pdist2(range,x0);
        %prob(d<=mu) = 0.15;
        
        prob(d<=mu & tell'<1) = 0.14;
        prob(obsIDx & d<=mu & between' == 0) = 0.86;


        M0(IDx{1}) = prob.*M0(IDx{1})./(prob.*M0(IDx{1})+(1-prob).*(1-M0(IDx{1})));
        M_new = reshape(M0,[cells cells]);
end
