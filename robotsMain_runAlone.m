% %map
clc; clear all; close all
seed = 6;
rng(seed)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initialize map
whichMap = "warehouse"; %[mapY, playground, maze, complex, warehouse]
[obs, M0] = setMap(whichMap);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

plt.gif = "yes";
plt.filename = "3Agents_POCMap";
xDim = 1:1:length(M0);
yDim = 1:1:length(M0);
Mo = M0';
fullMap = binaryOccupancyMap((Mo>0.85)');
gridSize = [length(xDim) length(yDim)];

% Create Meshgrid of coordinates
[X, Y] = meshgrid(xDim,yDim);
GridPoints = [X(:), Y(:)];
plt.X = X; plt.Y = Y; plt.xDim = xDim; plt.yDim = yDim;

% Initilize probability of each grid point (0.5)
M = 0.5*ones(length(xDim));
M0 = M(:);
Mtot = M0;
% Initial robot location
x0 = [3 3];
x1 = [3 4];
x2 = [4 4];
x3 = [33 35];
x = [x0; x1; x2];
agents = length(x);
count = ones(1,agents);
%Generate task
validator = validatorOccupancyMap(stateSpaceSE2);
task = generateTask(seed,fullMap,GridPoints,validator);
taskLocs = [55 55];
taskCompGlobal = 1000;
plt.agents = agents; plt.taskLocs = taskLocs;
% Define sensor radius
r = 10;

rbt = robots;
rbt = initialize(rbt,x,agents,M,r,obs,taskLocs,taskCompGlobal);

% Initialize known beginning states of all agents
pthObj = cell(agents,1);
pthObj_t = {};
rows = 1:length(rbt.M0k);

% Initialie weightings & thresholds
a = 1; b = 1;
eV = 200; 
varEps = 1;
thresh = 100;

steps = 10000;
for i = 1:steps
    tic
    %Store time values (for plotting)
    plt.x_t(:,:,i) = rbt.x;
    plt.M_t(:,:,i) = reshape(Mtot,gridSize);

    %If within range, agents communicate and combine maps and recent
    %information
    [rbt,shared] = communicate(rbt);

    %Check states
    rbt = stateChecker(rbt, Mtot, GridPoints, thresh,a,b,eV,varEps);
    rbt.taskComp = taskChecker(rbt.taskLocs,rbt.taskComp,rbt.x);

    %Evaluate
    for k = 1:agents
        x0 = rbt.x(k,:);
        M0 = rbt.M0k(:,k);
        
        if any(strcmp({'leader','follower','find'},rbt.newStates(k)))
            search = rbt.ag2find(k);
        elseif any(strcmp('exploit',rbt.newStates(k))) && rbt.taskFound(k)
            search = find(pdist2(rbt.x(k,:),rbt.taskLocs)<=rbt.mu);
        elseif any(strcmp('exploit',rbt.newStates(k)))
            search = rbt.ag2find(k);
        else
            search = k;
        end
        %Pull occupancy grid for agent of interest
        MGhostk = squeeze(rbt.MGhost(:,k,search));
        
        %Update individual map 
        [Entropy, G_Mx, G_My, M_new, rbt] = rbt.updateOccupancyMap(M0, k, GridPoints); 
            
        if rbt.newStates(k) == "find"
            [Entropy, Ghost_Mx, Ghost_My, Ghost_new, rbt] = rbt.updateOccupancyMap(MGhostk(:), k, GridPoints);
        else
            Ghost_new = MGhostk;
        end
        
        if i>72
            s = 1;
        end

        replan_ghost = count(k)>=size(pthObj{k},1)||(sum(sum(Ghost_new>0.85))>sum(sum(MGhostk>0.85))||shared(k));
        replan_norm = (count(k)>=size(pthObj{k},1)||rbt.timeAtState(k) == 0)||(sum(sum(M_new>0.85))>sum(sum(M0>0.85))||shared(k));

        if rbt.timeAtState(k) == 0 && rbt.newStates(k) == "find"
            count(k) = 1;
            rbt.target(k,:) = rbt.lastTarget(rbt.ag2find(k),:,k);
            pthObj{k} = mapAStarGrid(x0,rbt.target(k,:),Ghost_new);
        elseif replan_ghost && rbt.newStates(k) == "find" 
            %If finding, use matrix from when you last saw the agent
            count(k) = 1;
            rbt = findWaypoint(rbt, Ghost_Mx, Ghost_My, Ghost_new, search, GridPoints, k);
            pthObj{k} = mapAStarGrid(x0,rbt.target(k,:),Ghost_new);
        elseif (count(k)>=size(pthObj{k},1)||rbt.timeAtState(k) == 0) && rbt.newStates(k) == "meet" 
            count(k) = 1;
            rbt = findWaypoint(rbt, G_Mx, G_My, M_new, search, GridPoints, k);
            planTraveled = M_new>0.25;
            pthObj{k} = mapAStarGrid(x0,rbt.target(k,:),planTraveled);
        elseif rbt.newStates(k) == "meet" 
            %Pass if meeting up because we have found a valid path from
            %previous if statement
        elseif replan_norm
            count(k) = 1;
            rbt = findWaypoint(rbt, G_Mx, G_My, M_new, search, GridPoints, k);  
            pthObj{k} = mapAStarGrid(x0,rbt.target(k,:),M_new);  
        end
        
        %Store for plotting and increment path target
        plt.pthObj_t{k,i} = pthObj{k};
        plt.waypt_t(k,:,i) = rbt.target(k,:);
        count(k) = count(k) + 1;
        rbt.MGhost(:,k,search) = Ghost_new(:);
        rbt.M0k(:,k) = M_new(:);
        rbt.lastConnect = rbt.connected;

        if length(pthObj{k}(:,1))<2; count(k) = 1; end
        rbt.x(k,:) = [pthObj{k}(count(k),1) pthObj{k}(count(k),2)];
    end
    
    %Store calculated vals (for plotting)  
    [~,I] = max(abs(rbt.M0k-0.5),[],2);
    I = sub2ind(size(rbt.M0k),rows',I);
    Mtot = rbt.M0k(I);
    rbt.states = rbt.newStates;
    
    plt.states_t{i} = rbt.states;
    plt.task_t{i} = rbt.taskLocs;
    plt.taskComp_t{i} = rbt.taskComp./rbt.taskCompGlobal*100;
    
    if sum(rbt.frontierEmpty)==agents
        explorationComplete = i;
    end
    if all(rbt.taskComp<=0)
        taskComplete = i;
    end
    if sum(rbt.frontierEmpty)==agents && all(rbt.taskComp<=0)
        break
    end
    i
    %Plot in loop
    plotInline(plt,i,Mtot,gridSize)
toc;
rbt.time = rbt.time + 1;
end

%Make Video
plt.steps = i; 
makeVideo(plt)



