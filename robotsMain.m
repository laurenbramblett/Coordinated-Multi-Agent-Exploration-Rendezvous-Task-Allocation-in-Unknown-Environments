function [plt,totalTime] = robotsMain(plt,seed,obs,M0,x,runType,char)
%taskComplete,explorationComplete%Main function
rng(seed)
xDim = 1:1:length(M0);
yDim = 1:1:length(M0);
Mtmp = flip(rot90(M0,-1),2);
Mo = flipud(Mtmp);
fullMap = binaryOccupancyMap((Mo>0.85)); %was (Mo>0.85)' for Y
gridSize = [length(xDim) length(yDim)];
fakeAgents = 3;
% Create Meshgrid of coordinates
[X, Y] = meshgrid(xDim,yDim);
GridPoints = [X(:), Y(:)];
plt.X = X; plt.Y = Y; plt.xDim = xDim; plt.yDim = yDim;

% Initilize probability of each grid point (0.5)
M = 0.5*ones(length(xDim));
M0 = M(:);
Mtot = M0;

agents = size(x,1);
count = ones(1,agents);
%Generate task
validator = validatorOccupancyMap(stateSpaceSE2);
if char.taskGen == 1
    task = generateTask(seed,fullMap,GridPoints,validator);
else
    task = char.task;
end
% figure(); hold on; pcolor(Mtmp); colormap(flipud(bone)); plot(task(1),task(2),'r*'); hold off;

taskLocs = task; % exp3 = [14,2; 14, 14]; %Expp = [14 14]
taskCompGlobal = char.taskCompGlobal; %exp3 = [200; 200];
% pcolor(Mtmp); colormap(flipud(bone));
plt.agents = agents; plt.taskLocs = taskLocs;

% Define sensor radius
r = char.r; %Exp = 5 %mapX = 5;

rbt = robots;
rbt = initialize(rbt,x,agents,M,r,obs,taskLocs,taskCompGlobal);

% Initialize known beginning states of all agents
pthObj = cell(agents,1);
rows = 1:length(rbt.M0k);

% Initialize weightings & thresholds
a = char.a; b = char.b; %mapX_v2 = 30;
eV = char.eV; 
varEps = char.varEps; %Exp = 1000; mapX = 1;
thresh = char.thresh; %Exp = 20 %mapX = 15

steps = 10000;
for i = 1:steps
    tic
%     if i<3
%         continue
%     end
    %Store time values (for plotting)
    plt.x_t(:,:,i) = rbt.x;
    plt.M_t(:,:,i) = reshape(Mtot,gridSize);
    plt.Mk_t(:,:,i) = rbt.M0k;

    %If within range, agents communicate and combine maps and recent
    %information

    %Check states
    switch runType
        case "meetplan"
            [rbt,shared] = communicate(rbt);
            rbt = stateChecker_MP(rbt, Mtot, GridPoints, validator, thresh);
            rbt.taskComp = taskChecker(rbt.taskLocs,rbt.taskComp,rbt.x);
        case "SR"
            rbt = stateChecker_One(rbt, Mtot);
            rbt.centroid = 1; shared = 0; rbt.partition = ones(length(M0),1);
            rbt.taskComp = taskChecker_SR(rbt.taskLocs,rbt.taskComp,rbt.x,fakeAgents);
        otherwise
            [rbt,shared] = communicate(rbt);
            rbt = stateChecker(rbt, Mtot, GridPoints,thresh,a,b,eV,varEps);
            rbt.taskComp = taskChecker(rbt.taskLocs,rbt.taskComp,rbt.x);
    end
    

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
        
        replan_ghost = count(k)>=size(pthObj{k},1)||(sum(sum(Ghost_new>0.85))>sum(sum(MGhostk>0.85))||shared(k));
        replan_norm = (count(k)>=size(pthObj{k},1)||rbt.timeAtState(k) == 0)||(sum(sum(M_new>0.85))>sum(sum(M0>0.85))||shared(k));

        if rbt.timeAtState(k) == 0 && rbt.newStates(k) == "find" && ~replan_ghost
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
            pthObjNoRisk = mapAStarGrid(x0,rbt.target(k,:),planTraveled);
            pthObjRisk = mapAStarGrid(x0,rbt.target(k,:),M_new);
            rzDecision = rzDecide(pthObjRisk,varEps,M_new,size(pthObjNoRisk,1),0,0);
            if rzDecision
                pthObj{k} = pthObjRisk;
            else
                pthObj{k} = pthObjNoRisk;
            end
%         elseif rbt.newStates(k) == "meet" 
%             %Pass if meeting up because we have found a valid path from
%             %previous if statement
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
    plt.partition_t{i} = rbt.partition;
    plt.centroid_t{i} = rbt.centroid;
    
    if (sum(rbt.frontierEmpty)==agents || (rbt.connected>0 && sum(rbt.frontierEmpty)>0))...
            && ~exist('explorationComplete','var')|| sum(sum(M0>0.15 & M0<0.85))<=agents
        explorationComplete = i;
    end
    if all(rbt.taskComp<=0) && ~exist('taskComplete','var')
        taskComplete = i;
    end
    if exist('explorationComplete','var') && exist('taskComplete','var')
        break
    end
    i
    %Plot in loop
    if char.plotInline
        plotInline(plt,i,Mtot,gridSize)
    end
toc;
rbt.time = rbt.time + 1;
end
totalTime = i;
plt.steps = i;
end



