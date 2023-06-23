function [fp,noFp] = frontierPoint(G_Mx,G_My,GridPoints,x,k,M0,search,C,centroid,partition)
        noFp = 0;
        S = sqrt(G_Mx.^2 + G_My.^2);
        S([1 length(G_Mx)],:) = 0;
        S(:,[1 length(G_Mx)]) = 0;
        indicies = find(M0>0.85);
        [finds, ~,~] = neighbourND(indicies,size(S));
        finds = [finds indicies];
        finds = unique(finds);
        place = find(finds == 0);
        if ~isempty(place)
            finds = finds(place~=1:end);
        end
        S(finds) = 0;
        s = S(:);
        frontier = find(s>0.1);
        
        distPartition = pdist2(C(centroid(search),:),GridPoints(frontier,:))';
        partitionLogi = partition(frontier)~=centroid(search);
        distMat = partitionLogi.*distPartition;
        distSelf = pdist2(x(k,:),GridPoints(frontier,:))';
        [val,idx] = min(200.*distMat+distSelf);
        idx = sort(idx);
        if isempty(frontier)
            fp = x(k,:);
        else
            fp = GridPoints(frontier(idx(1)),:);
        end
end