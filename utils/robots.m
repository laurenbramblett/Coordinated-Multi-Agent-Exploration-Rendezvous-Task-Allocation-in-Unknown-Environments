classdef robots
   properties
       agents
       x
       states
       count
       pthObj
       newStates
       last_seen
       lastTarget
       M0k
       MGhost
       timeAtState
       waitTime
       maxPathTime
       target
       lastSeenTime
       lastSeenTimeTot
       ag2find
       centroid
       cLoc
       meet
       r
       mu
       connected
       lastConnect
       partition
       rendezvous
       taskFound
       time
       gridSize
       taskLocs
       taskType
       taskAssigned
       taskComp
       obs
       whichTask
       taskDecided
       exploreTime
       taskCompGlobal
       localTaskLocs
       localTaskAssigned
       frontierEmpty
   end
   methods
        function obj = initialize(obj,x,agents,M,r,obs,taskLocs,taskCompGlobal)
            M0 = M(:);
            obj.agents = agents;
            obj.x = x;
            obj.count = ones(1,agents);
            obj.pthObj = cell(agents,1);
            obj.states = repmat("explore",[agents,1]);
            obj.newStates = obj.states;
            obj.last_seen = repmat(x,[1,1,agents]); %All robots begin with perfect information
            obj.lastTarget = obj.last_seen;
            obj.M0k = repmat(M0,[1,agents]);
            obj.MGhost = repmat(M0,[1,agents,agents]); %All begin with each others map
            obj.timeAtState = zeros(agents,1);
            obj.waitTime = obj.timeAtState;
            obj.maxPathTime = obj.waitTime;
            obj.target = zeros(agents,2);
            obj.lastSeenTime = zeros(agents);
            obj.lastSeenTimeTot = obj.lastSeenTime;
            obj.ag2find = ones(agents,1);
            obj.centroid = zeros(agents,1);
            obj.cLoc = zeros(agents,2);
            obj.meet = ones(agents,1)*-10;
            obj.r = r;
            obj.mu = r*3/4;
            obj.connected = 0;
            obj.lastConnect = 0;
            obj.partition = zeros(length(obj.M0k),1);
            obj.rendezvous = zeros(1,2);
            obj.taskFound = zeros(agents,1);
            obj.time = 0;  
            obj.taskLocs = taskLocs;
            obj.taskAssigned
            obj.gridSize = size(M);
            obj.obs = obs;
            obj.taskDecided = zeros(size(taskLocs,1),1);
            obj.taskFound = zeros(agents,1);
            obj.exploreTime = zeros(agents,length(taskLocs));
            obj.taskCompGlobal = taskCompGlobal;
            obj.taskComp = taskCompGlobal;
            obj.taskType = strings(agents,1);
            obj.time = 1;
            obj.localTaskLocs = []; obj.localTaskAssigned = [];
            obj.whichTask = cell(agents,1);
            obj.frontierEmpty = zeros(agents,1);
        end
       
    
        function [obj,shared] = communicate(obj)
            rows = 1:length(obj.M0k);
            distOther = squareform(pdist(obj.x))<obj.mu;
            bins = conncomp(graph(distOther));
            comps = 1:numel(bins);
            shared = zeros(length(obj.x),1);
            %maxPathTime = zeros(length(x),1);
        %     broken = shared;

            for k = 1:obj.agents
                %Store old M0k obstacles to see if any new obstacles are shared
                %(for replanning)
                Mlast = obj.M0k(:,k)>0.85;
                %Bid and sync disconnected information
                inbins = find(bins(k) == bins);
                nonbin = setdiff(comps,inbins);
                if ~isempty(nonbin)
                    [~,idx] = min(obj.lastSeenTime(inbins,nonbin),[],1);
                    bidWin = inbins(idx);
                    self = repmat(k,1,length(bidWin));
                    if obj.states(k) ~= "find"
                        obj.MGhost(:,self,nonbin) = obj.MGhost(:,bidWin,nonbin);
                    end
                    obj.last_seen(nonbin,:,self) = obj.last_seen(nonbin,:,bidWin);
                    obj.lastTarget(nonbin,:,self) = obj.lastTarget(nonbin,:,bidWin);
                    obj.lastSeenTime(self,nonbin) = obj.lastSeenTime(bidWin,nonbin);
                end
        %         broken(k) = any(lastSeenTime(k,:) == 1);
                %Update last seen
                obj.last_seen(inbins,:,k) = obj.x(inbins,:);
                obj.lastTarget(inbins,:,k) = obj.target(inbins,:);
                %Increment time either seen or not seen
                obj.lastSeenTime(k,nonbin) = obj.lastSeenTime(k,nonbin)+1; 
                obj.lastSeenTime(k,inbins) = zeros([1, length(inbins)]);
                %obj.lastSeenTimeTot(k,inbins) = ones([1, length(inbins)]).*time;
                %Communicate occupancy grid cells that are updated
                [~,I] = max(abs(obj.M0k(:,inbins)-0.5),[],2);
                I = sub2ind(size(obj.M0k(:,inbins)),rows',I);
                Mtotk = obj.M0k(:,inbins);
                obj.M0k(:,k) = Mtotk(I);
                shared(k) = sum(obj.M0k(:,k)>0.85)>sum(Mlast);
                %Set occupancy ghost grid to map frontier for robot designated to
                %that cluster
                obj.MGhost(:,k,inbins) = repmat(obj.M0k(:,k),[1,length(inbins)]);
                GhostStates = ismember(obj.states(inbins),["leader","find"]);
                if obj.states(k) == "find" && sum(GhostStates)>1
                    %Defining interactions during find phase
                    ghostBins = inbins(GhostStates);
                    combGhost = zeros(length(obj.MGhost),length(ghostBins));
                    for g = 1:length(ghostBins)
                        combGhost(:,g) = obj.MGhost(:,ghostBins(g),obj.ag2find(g));
                    end
                    %otherGhost = otherGhost(
                    [~,I] = max(abs(combGhost-0.5),[],2);
                    I = sub2ind(size(combGhost),rows',I); 
                    obj.MGhost(:,k,obj.ag2find(k)) = combGhost(I);
                end
            end
        end
        
        function obj = stateChecker(obj, M, GridPoints, thresh,a,b,eV,varEps)
            set = squareform(pdist(obj.x));
            %If any are connecting the other agents (ie a middle agent) then they are
            %still communicating
            connect = any(all(set<=obj.mu,2));
            %Rendezvous point using center of mass and validity check
            M0 = M;
            M = reshape(M,[sqrt(length(M)),sqrt(length(M))])>=0.15;
            sub_obs = find(M == 1);
            [row_o,col_o] = ind2sub(size(M),sub_obs);
            %Define global state trigger
            meetThresh = 2*thresh; exThresh = thresh;

            if mod(obj.time,meetThresh)<=mod(obj.time,exThresh)
                globalState = "explore";
            else
                globalState = "meet";
            end

            if connect
                obj.connected = obj.connected + 1;
                if globalState == "explore"

                end
                if obj.connected == 1
                    unk = M0>0.15 & M0<=0.85;
                    try
                        [idx,C] = kmeans(GridPoints(unk,:),length(obj.x));
                        obj.partition = zeros(numel(unk),1);
                        obj.partition(unk) = idx;
                        C = [C(:,2) C(:,1)];
                        obj.cLoc = C;
                        a_q = 1:size(C,1); c_q = a_q;
                        chosen = zeros(length(obj.x),1);
                        %Bidding set for centroid
                        for i = 1:obj.agents 
                            dist = pdist2(obj.x(a_q,:),C(c_q,:));
                            [~,idx] = min(dist,[],'all','linear');
                            [a,c] = ind2sub(size(dist),idx);
                            a = a_q(a); c = c_q(c);
                            a_q = setdiff(a_q,a);
                            c_q = setdiff(c_q,c);
                            chosen(a) = c;
                        end
                        obj.centroid = chosen;
                    catch
                        obj.newStates = repmat("exploit",[obj.agents,1]);
                    end
                elseif obj.connected >= 3 && ~all(strcmp(obj.states,'exploit'))
                    obj.time = 0;
                    obj.newStates = repmat("explore",[obj.agents,1]);
                    %Rendezvous location
                    unk_part = obj.partition(obj.partition~=0);
                    [C,~,ic] = unique(unk_part);
                    a_counts = accumarray(ic,1);
                    value_counts = [C, a_counts];
                    obj.rendezvous = round(sum(obj.cLoc.*value_counts(:,2),1)./length(unk_part),0);
%                     isValid = isStateValid(validator,[obj.rendezvous 0]);
                    isValid = ~ismember(obj.rendezvous,[col_o,row_o],'rows');
                    checked = [0 0]; rad = 1;
                    while ~isValid
                        IDx = rangesearch(GridPoints,obj.rendezvous,rad,'Distance','euclidean');
                        search = GridPoints(IDx{1},:);
                        search = setdiff(search,checked,'rows');
                        if ~isempty(search)
                            for j = 1:length(search)
%                                 isValid = isStateValid(validator,[search(j,:) 0]);
                                isValid = ~ismember(obj.rendezvous,[col_o,row_o],'rows');
                                if isValid
                                    obj.rendezvous = search(j,:);
                                    break
                                else
                                    checked = [checked; search(j,:)];
                                end
                            end
                            
                        else
                            obj.rendezvous = obj.x(1,:);
                            break
                        end
                    end
%                     obj.rendezvous = [obj.rendezvous(2),obj.rendezvous(1)];
                end
            else
                obj.connected = 0;
            end
            %%%%%%%%%%%%%%Rendezvous&KmeansEnd%%%%%%%%%%

            globalState
            meetLead = find(obj.states == "meet" & pdist2(obj.x,obj.rendezvous)<=obj.mu,1);
            
            %Allocate tasks if meetup time is up
            if ((mod(obj.time,meetThresh) == meetThresh-1 && ~isempty(meetLead))) ||...
                    (connect && any(obj.taskType == "rz_tf"))
                obj.time = 0;
                if isempty(meetLead) && connect; meetLead = find(obj.taskType == "rz_tf"); end
                occMap = reshape(obj.M0k(:,meetLead(1)),obj.gridSize);
                Gx = [1 0 -1; 2 0 -2; 1 0 -1];
                Gy = [1 2 1; 0 0 0; -1 -2 -1];

                G_Mx = filter2(Gx,occMap);
                G_My = filter2(Gy,occMap);
                %FIX
                obj = obj.allocateTasks(connect,G_Mx,G_My,a,b,eV,...
                    meetThresh,exThresh,GridPoints);
            end
            for k = 1:obj.agents
                M0 = reshape(obj.M0k(:,k),obj.gridSize);
                if ~strcmp(obj.taskType(k), "rz_tf")
                    obj.taskFound(k) = any(pdist2(obj.taskLocs,obj.x(k,:))<=obj.mu & obj.taskComp>0);
                    obj.whichTask{k} = find(pdist2(obj.taskLocs,obj.x(k,:))<=obj.mu);
                end

                if obj.taskFound(k) && obj.states(k) == "explore" && obj.taskComp(obj.whichTask{k})>0
                    obj.newStates(k) = "exploit";
                    foundIdx = pdist2(obj.taskLocs,obj.x(k,:))<=obj.mu;
                    obj.exploreTime(k,foundIdx) = mod(obj.time,exThresh);
                elseif obj.taskFound(k) && (obj.states(k) == "meet" ||...
                        mod(obj.time,exThresh) == 0) && obj.taskComp(obj.whichTask{k})>0 && obj.taskDecided(obj.whichTask{k}) < 1
                    pthToRz = mapAStarGrid(obj.x(k,:),obj.rendezvous,M0);
                    rzDecision = rzDecide(pthToRz,varEps,M0,obj.exploreTime(k,obj.taskFound(k)),obj.taskComp(obj.whichTask{k}),eV);
                    obj.newStates(k) = "meet";
                    obj.taskType(k) = "rz_tf";
                    if ~rzDecision || connect
                        obj.newStates(k) = "exploit";
                    end
                    taskLocated = pdist2(obj.taskLocs,obj.x(k,:))<=obj.mu;
                    obj.taskDecided(taskLocated) = 1;

                elseif obj.taskFound(k) && obj.states(k) == "find" && obj.taskComp(obj.whichTask{k})>0
                    obj.newStates(k) = "exploit";
                elseif isempty(obj.taskType)
                    obj.newStates(k) = globalState;
                elseif obj.taskType(k) == "rz"
                    obj.newStates(k) = "exploit";
                    obj.taskType(k) = "na";
                    obj.ag2find(k) = obj.localTaskAssigned(k);
                elseif obj.taskType(k) == "lost"
                    obj.newStates(k) = "leader";
                    obj.taskType(k) = "na";
                    last_seenk = squeeze(obj.last_seen(:,:,k));
                    obj.ag2find(k) = find(all(last_seenk == obj.localTaskLocs(obj.localTaskAssigned(k),:),2));
                elseif obj.states(k) == "leader" 
                    last_seenk = squeeze(obj.last_seen(:,:,k));
                    if pdist2(obj.x(k,:),last_seenk(obj.ag2find(k),:))<1
                        obj.newStates(k) = "find";
                    end
                elseif obj.taskComp(obj.whichTask{k})<=0
                    obj.newStates(k) = globalState;
                elseif obj.states(k) == "find" && globalState == "meet"
                    obj.newStates(k) = "meet";
                elseif ~ismember(obj.states(k), ["explore","meet"])
                    obj.newStates(k) = obj.states(k);
                else
                    obj.newStates(k) = globalState;
                end

                if obj.newStates(k) == obj.states(k)
                    obj.timeAtState(k) = obj.timeAtState(k) + 1;
                else
                    obj.timeAtState(k) = 0;
                end
            end
        end

        function obj = allocateTasks(obj,connect,G_Mx,G_My,a,b,eV,meetThresh,exThresh,GridPoints)
            agentIDs = 1:obj.agents;
            meetBool = (obj.states == "meet")& pdist2(obj.x,obj.rendezvous)<=obj.mu;
            if connect
                meetBool = logical(ones(obj.agents,1));
            end
            meetAgents = agentIDs(meetBool);
            meetLead = meetAgents(1);
            M0 = obj.M0k(:,meetLead);
            lastSeenMeet = squeeze(obj.last_seen(:,:,meetLead));

            rzTasksFound = unique(cell2mat(obj.whichTask(meetAgents)));
            if ~isempty(rzTasksFound)
                rzTasksFound = rzTasksFound(obj.taskComp(rzTasksFound)>0);
            end
            
            rzTasks = obj.taskLocs(rzTasksFound,:); %Tasks from rz agents
            rzTaskComp = obj.taskComp(rzTasksFound); %Completion shared from rz agents
            lostAgents = find(obj.lastSeenTime(meetLead,:)>0);
            lostTasks = lastSeenMeet(lostAgents,:); %Last location of lost agents
            lostMult = floor(obj.lastSeenTime(meetLead,lostAgents)./meetThresh)+1; %How many times haven't we seen agent
            lostComp = eV - lostMult*(meetThresh-exThresh) - exThresh;
            obj.localTaskLocs = [rzTasks; lostTasks];
            rzTaskNum = size(rzTasks,1); %lostTaskNum = length(lostTasks);
            taskCompTotal = [rzTaskComp,lostComp];
            taskNum = length(taskCompTotal);
            ka = obj.agents-taskNum;

            combos = combinator(ka+1,taskNum)-1;
            validCombos = combos(sum(combos,2)<=ka,:);
            comboNum = size(validCombos,1);
            for k = 1:length(meetAgents)
                meetk = meetAgents(k);
                [fp(k,:),~] = frontierPoint(G_Mx,G_My,GridPoints,obj.x,meetk,M0,meetk,obj.cLoc,obj.centroid,obj.partition);
                pthObjMeet = mapAStarGrid(obj.x(meetk,:),fp(k,:),reshape(M0,obj.gridSize));
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
                distToTask = pdist2(obj.x(meetAgents,:),obj.localTaskLocs);
                taskEnum = 1:length(taskAlloc);
                taskBool = taskAlloc>0;
                obj.localTaskAssigned = zeros(obj.agents,1);
                obj.taskType = repmat("na",[obj.agents,1]);
                for k = 1:length(meetAgents)
                    taskAvail = taskEnum(taskBool);
                    [~,idx] = min(distToTask(k,taskAvail));

                    if isempty(idx)
                        obj.localTaskAssigned(meetAgents(k)) = 0;
                        obj.taskType(meetAgents(k)) = "na";
                    else
                        obj.localTaskAssigned(meetAgents(k)) = taskAvail(idx);
                        if obj.localTaskAssigned(meetAgents(k))<=length(rzTasks)
                            obj.taskType(meetAgents(k)) = "rz";
                        else
                            obj.taskType(meetAgents(k)) = "lost";
                        end
                    end
                    if obj.localTaskAssigned(meetAgents(k)) > 0
                        taskAlloc(obj.localTaskAssigned(meetAgents(k))) = taskAlloc(obj.localTaskAssigned(meetAgents(k)))-1;
                    end
                    taskBool = taskAlloc>0;
                end
            end

        end

        function obj = findWaypoint(obj, G_Mx, G_My, M0, search, GridPoints, k)
            switch obj.newStates(k)
                case {'explore', 'find'}
                    S = sqrt(G_Mx.^2 + G_My.^2);
                    S([1 size(G_Mx,1)],:) = 0;
                    S(:,[1 size(G_Mx,2)]) = 0;
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
                    distPartition = pdist2(obj.cLoc(obj.centroid(search),:),GridPoints(frontier,:))';
                    partitionLogi = obj.partition(frontier)~=obj.centroid(search);
                    distMat = partitionLogi.*distPartition;
                    distSelf = pdist2(obj.x(k,:),GridPoints(frontier,:))';
                    [~,idx] = min(1.*distMat+distSelf);
                    idx = sort(idx);
                    if isempty(frontier)
                        obj.target(k,:) = obj.rendezvous;
                        obj.frontierEmpty(k) = 1;
                    else
                        obj.target(k,:) = GridPoints(frontier(idx(1)),:);
                    end
                case 'meet'
                    obj.target(k,:) = obj.rendezvous;
                case {'follower','leader'}
                    obj.target(k,:) = obj.last_seen(search,:,k);
                case 'exploit'
                    obj.target(k,:) = obj.taskLocs(search,:);
                otherwise
                    obj.target(k,:) = obj.target(k,:);
            end
        end

        function [Entropy, G_Mx, G_My, M_new, obj] = updateOccupancyMap(obj, M0, k, GridPoints) 
            % Find all points within range of sensor
            x0 = obj.x(k,:)-0.5;
            IDx = rangesearch(GridPoints,x0,obj.r,'Distance','euclidean');
            range = GridPoints(IDx{1},:);
            prob = ones(length(IDx{1}),1)*0.5;
            obsIDx = ismember(range,obj.obs,'rows');
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
            
            prob(d<=obj.mu & tell'<1) = 0.14;
            prob(obsIDx & d<=obj.mu & between' == 0) = 0.86;


            M0(IDx{1}) = prob.*M0(IDx{1})./(prob.*M0(IDx{1})+(1-prob).*(1-M0(IDx{1})));
            M_new = reshape(M0,obj.gridSize);

            Interest = sum(M0 == 0.5);

            E = M_new*0.999;
            Entropy = -sum(sum(E.*log2(E)+(1-E).*log2(1-E)));

            % Find gradient of probability
            Gx = [1 0 -1; 2 0 -2; 1 0 -1];
            Gy = [1 2 1; 0 0 0; -1 -2 -1];

            G_Mx = filter2(Gx,M_new);
            G_My = filter2(Gy,M_new);
        end

        function obj = stateChecker_MP(obj, M, GridPoints, validator, thresh)
            set = squareform(pdist(obj.x));
            %If any are connecting the other agents (ie a middle agent) then they are
            %still communicating
            connect = any(all(set<=obj.mu,2));
            %Rendezvous point using center of mass and validity check
            M0 = M;
            M = reshape(M,[sqrt(length(M)),sqrt(length(M))])>0.14;
            sub_obs = find(M == 1);
            [row_o,col_o] = ind2sub(size(M),sub_obs);
%             M = M';
%             validator.Map = binaryOccupancyMap(M);
            %Define global state trigger
            meetThresh = 2*thresh; exThresh = thresh;

            if mod(obj.time,meetThresh)<=mod(obj.time,exThresh)
                globalState = "explore";
            else
                globalState = "meet";
            end

            if connect
                obj.connected = obj.connected + 1;
                if globalState == "explore"
                    
                end
                if obj.connected == 1
                    unk = M0>0.15 & M0<=0.85;
                    try
                        [idx,C] = kmeans(GridPoints(unk,:),length(obj.x));
                        obj.partition = zeros(numel(unk),1);
                        obj.partition(unk) = idx;
                        C = [C(:,2) C(:,1)];
                        obj.cLoc = C;
                        a_q = 1:size(C,1); c_q = a_q;
                        chosen = zeros(length(obj.x),1);
                        %Bidding set for centroid
                        for i = 1:obj.agents 
                            dist = pdist2(obj.x(a_q,:),C(c_q,:));
                            [~,idx] = min(dist,[],'all','linear');
                            [a,c] = ind2sub(size(dist),idx);
                            a = a_q(a); c = c_q(c);
                            a_q = setdiff(a_q,a);
                            c_q = setdiff(c_q,c);
                            chosen(a) = c;
                        end
                        obj.centroid = chosen;
                    catch
                        obj.newStates = repmat("exploit",[obj.agents,1]);
                    end
                elseif obj.connected >= 5 && ~all(strcmp(obj.states,'exploit'))
                    obj.time = 0;
                    obj.newStates = repmat("explore",[obj.agents,1]);
                    %Rendezvous location
                    unk_part = obj.partition(obj.partition~=0);
                    [C,~,ic] = unique(unk_part);
                    a_counts = accumarray(ic,1);
                    value_counts = [C, a_counts];
                    obj.rendezvous = round(sum(obj.cLoc.*value_counts(:,2),1)./length(unk_part),0);
%                     isValid = isStateValid(validator,[obj.rendezvous 0]);
                    isValid = ~ismember(obj.rendezvous,[col_o,row_o],'rows');
                    checked = [0 0]; rad = 1;
                    while ~isValid
                        IDx = rangesearch(GridPoints,obj.rendezvous,rad,'Distance','euclidean');
                        search = GridPoints(IDx{1},:);
                        search = setdiff(search,checked,'rows');
                        if ~isempty(search)
                            for j = 1:length(search)
%                                 isValid = isStateValid(validator,[search(j,:) 0]);
                                isValid = ~ismember(obj.rendezvous,[col_o,row_o],'rows');
                                if isValid
                                    obj.rendezvous = search(j,:);
                                    break
                                else
                                    checked = [checked; search(j,:)];
                                end
                            end
                        else
                            obj.rendezvous = obj.x(1,:);
                            break
                        end
                    end
                end
            else
                obj.connected = 0;
            end
            %%%%%%%%%%%%%%Rendezvous&KmeansEnd%%%%%%%%%%

            globalState
            meetLead = find(obj.states == "meet",1);
            %Allocate tasks if meetup time is up
            
            for k = 1:obj.agents
                if ~strcmp(obj.taskType(k), "rz_tf")
                    obj.taskFound(k) = any(pdist2(obj.taskLocs,obj.x(k,:))<=obj.mu & obj.taskComp>0);
                    obj.whichTask{k} = find(pdist2(obj.taskLocs,obj.x(k,:))<=obj.mu);
                end

                if obj.taskFound(k) && obj.taskComp(obj.whichTask{k})>0
                    obj.newStates(k) = "exploit";
                    foundIdx = pdist2(obj.taskLocs,obj.x(k,:))<=obj.mu;
                    obj.exploreTime(k,foundIdx) = mod(obj.time,exThresh);
                elseif isempty(obj.taskType)
                    obj.newStates(k) = globalState;
                elseif obj.taskComp(obj.whichTask{k})<=0
                    obj.newStates(k) = globalState;
                elseif ~ismember(obj.states(k), ["explore","meet"])
                    obj.newStates(k) = obj.states(k);
                else
                    obj.newStates(k) = globalState;
                end

                if obj.newStates(k) == obj.states(k)
                    obj.timeAtState(k) = obj.timeAtState(k) + 1;
                else
                    obj.timeAtState(k) = 0;
                end
            end
        end
        function obj = stateChecker_One(obj, M)
            obj.partition = ones(length(M(:)));
            obj.cLoc = [1 1];
            obj.centroid = ones(obj.agents,1);
            if pdist2(obj.taskLocs,obj.x)<=obj.mu && obj.taskComp>0
                obj.newStates = "exploit";
            else
                obj.newStates = "explore";
            end
            obj.rendezvous = obj.x;
            if obj.newStates == obj.states
                obj.timeAtState = obj.timeAtState + 1;
            else
                obj.timeAtState = 0;
            end
            obj.states = obj.newStates;
        end
    end
end
