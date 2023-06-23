function task = generateTask(seed,map,GridPoints,validator)
    rng(seed)
    lims = map.GridSize;
    task = [randi(lims(1)),randi(lims(2))];
    validator.Map = map;
    isValid = isStateValid(validator,[task 0]);
    search = []; checked = [0 0]; r = 1;
    while ~isValid
        
        IDx = rangesearch(GridPoints,task,r,'Distance','euclidean');
        search = GridPoints(IDx{1},:);
        search = setdiff(search,checked,'rows');

        for j = 1:length(search)
            isValid = isStateValid(validator,[search(j,:) 0]);
            if isValid
                task = [search(j,:)];
                break
            else
                checked = [checked; search(j,:)];
            end
        end
        r = r + 1;
    end
    task = [task(1), task(2)];
end
