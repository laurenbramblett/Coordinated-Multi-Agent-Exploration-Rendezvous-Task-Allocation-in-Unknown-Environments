function [taskComp] = taskChecker(taskLocs,taskComp,x)
    for l = 1:size(taskLocs,1)
        tmp = all(x == taskLocs(l,:),2);
        if sum(tmp)>0
            taskComp(l) = taskComp(l)-sum(tmp);
        end
    end
end
        