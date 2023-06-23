function [taskAssigned, allTaskLocs, taskType] = allocateTasks(agents,states,...
    x,lastSeenTime,last_seen,whichTask,G_Mx,G_My,GridPoints,M0,C,centroid,partition,...
    taskLocs,taskComp,meetThresh,exThresh)
taskAssigned = []; taskType = [];
eV = 1000; a = 1; b = 5;
agentIDs = 1:agents;
meetBool = (states == "meet");
meetAgents = agentIDs(meetBool);
meetLead = meetAgents(1);
lastSeenMeet = squeeze(last_seen(:,:,meetLead));

rzTasksFound = cell2mat(whichTask(meetAgents));

rzTasks = taskLocs(rzTasksFound,:); %Tasks from rz agents
rzTaskComp = taskComp(rzTasksFound); %Completion shared from rz agents
lostAgents = find(lastSeenTime(meetLead,:)>0);
lostTasks = lastSeenMeet(lostAgents,:); %Last location of lost agents
lostMult = floor(lastSeenTime(meetLead,lostAgents)./meetThresh)+1; %How many times haven't we seen agent
lostComp = eV - lostMult*(meetThresh-exThresh) - exThresh;
allTaskLocs = [rzTasks; lostTasks];
rzTaskNum = size(rzTasks,1); %lostTaskNum = length(lostTasks);
taskCompTotal = [rzTaskComp,lostComp];
taskNum = length(taskCompTotal);
ka = agents-taskNum;

combos = combinator(ka+1,taskNum)-1;
validCombos = combos(sum(combos,2)<=ka,:);
comboNum = size(validCombos,1);
for k = 1:length(meetAgents)
    meetk = meetAgents(k);
    fp(k,:) = frontierPoint(G_Mx,G_My,GridPoints,x,meetk,M0,meetk,C,centroid,partition);
    pthObjMeet = mapAStarGrid(x(meetk,:),fp(k,:),reshape(M0,size(G_Mx)));
    pthLength(k) = length(pthObjMeet(:,1));
end

[~,sortIdx] = sort(pthLength);
for n = 1:comboNum
    ke = ka-sum(validCombos(n,:));
    timeSavings = -sum(a*taskCompTotal./(validCombos(n,:)+1));
    entropyIncr = 0;
    if ke>0
        entropyIncr = b*(exThresh-pthLength(sortIdx(ke)))*ke;
    end
    totalObj(n) = timeSavings + entropyIncr;
end

if comboNum>0 %One task
    [~,allocIdx] = max(totalObj); %Max of the combos
    taskAlloc = validCombos(allocIdx,:); %Allocate to tasks (Have to add +1 here if the finding agent rzs FIX)
    if rzTaskNum>0; taskAlloc(1:rzTaskNum) = taskAlloc(1:rzTaskNum) + 1; end
    distToTask = pdist2(x(meetAgents,:),allTaskLocs);
    taskEnum = 1:length(taskAlloc);
    taskBool = taskAlloc>0;
    taskAssigned = zeros(agents,1);
    taskType = repmat("na",[agents,1]);
    for k = 1:length(meetAgents)
        taskAvail = taskEnum(taskBool);
        [~,idx] = min(distToTask(k,taskAvail));
        
        if isempty(idx)
            taskAssigned(meetAgents(k)) = 0;
            taskType(meetAgents(k)) = "na";
        else
            taskAssigned(meetAgents(k)) = taskAvail(idx);
            if taskAssigned(meetAgents(k))<=length(rzTasks)
                taskType(meetAgents(k)) = "rz";
            else
                taskType(meetAgents(k)) = "lost";
            end
        end
        if taskAssigned(meetAgents(k)) > 0
            taskAlloc(taskAssigned(meetAgents(k))) = taskAlloc(taskAssigned(meetAgents(k)))-1;
        end
        taskBool = taskAlloc>0;
    end
end


end
