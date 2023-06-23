function [taskComp] = taskChecker_SR(taskLocs,taskComp,x,agents)
    for l = 1:size(taskLocs,1)
        tmp = all(x == taskLocs(l,:),2)*agents;
        if sum(tmp)>0
            taskComp(l) = taskComp(l)-sum(tmp);
        end
    end
end
        