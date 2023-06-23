function [plt,F] = makeVideo(plt,conflict_correction)
agents = size(plt.x_t,1);
GridPoints = [plt.X(:),plt.Y(:)];
plt2 = plt;
if conflict_correction
for p = 2:plt.steps
    M0 = squeeze(plt.M_t(:,:,p));
    pos = squeeze(plt.x_t(:,:,p));
    last_pos = squeeze(plt2.x_t(:,:,p-1));
    if p>=206
        s = 1;
    end
    [~,c_idx] = sort(sqrt((pos(:,2)-last_pos(:,2)).^2 + (pos(:,1)-last_pos(:,1)).^2));
    for k = 2:agents
        imp = c_idx(k);
        conflicts = ismember(pos(c_idx(1:k-1),:),pos(imp,:),'rows');
        if any(conflicts)
            [pts,D] = rangesearch(GridPoints,pos(imp,:),2,'Distance','euclidean');
%             [pts1,~] = rangesearch(GridPoints,pos(imp,:),1,'Distance','euclidean');
%             [diffpts,ia] = setdiff(pts{1},pts1{1});
            D = D{1};
            [c_r,c_c] = ind2sub(size(M0),pts{1});
            [tmp_cands,ia] = setdiff([c_c',c_r'],pos(imp,:),'rows');
            D = D(ia);
%             tmp_cands = [c_c',c_r'];

            dist_curr_pos = pdist2(tmp_cands,last_pos(imp,:));
            weighted = D'+dist_curr_pos;
            isValid = 0; c = 1;
            while ~isValid
                [ordered,idx] = sort(weighted);
                pos_candidate = tmp_cands(idx(c),:);
                isValid = M0(pos_candidate(2),pos_candidate(1))<0.3;
                c = c+1;
            end
            M0(pos_candidate(2),pos_candidate(1)) = 1;
            pos(imp,:) = [pos_candidate(1),pos_candidate(2)];
        end
        plt2.x_t(imp,:,p) = pos(imp,:);
    end
    p
end
end
plt = plt2;
f = figure('Position',[100 100 1280/2 720/2]);
% f = figure();
set(f, 'MenuBar', 'none');

agents = size(plt.x_t,1);
cm = cbrewer('qual','Accent',agents);
blk = 64/255;
cm_unk = cm;

%  axis square; % this ensures that getframe() returns a consistent size

for j = 1:plt.steps-1
     subplot(1,3,[1 2]);
    heglines = [];
    hold on
%     axis square;
    h = pcolor(plt.X-1,plt.Y-1,squeeze(plt.M_t(:,:,j))); colormap(flipud(bone)); clim([0,1])
    set(h,'EdgeColor','none')
    for k = 1:agents
        ag_plot = plot(plt.x_t(k,1,j)-0.5,plt.x_t(k,2,j)-0.5,'Marker','o','MarkerFaceColor',cm(k,:),'MarkerEdgeColor',[64,64,64]./255,'Color',cm(k,:),'Linestyle', 'none');
        plot(plt.waypt_t(k,1,j)-0.5,plt.waypt_t(k,2,j)-0.5,'r*')
        path = plot(plt.pthObj_t{k,j}(:,1)-0.5,plt.pthObj_t{k,j}(:,2)-0.5,'-r');
        heglines = [heglines ag_plot(1)];
        plotNames{k} = strcat("Agent ",num2str(k));
    end
    tasklines = [];
    taskColors = [55,126,184;
                  152,78,163]./255;
    for t = 1:size(plt.task_t{1,j},1)
        task = plot(plt.task_t{1,j}(t,1)-0.5,plt.task_t{1,j}(t,2)-0.5,'Marker','d','MarkerFaceColor',taskColors(t,:),'Color',taskColors(t,:),'MarkerEdgeColor','k','Linestyle', 'none','LineWidth',1.25);
        tasklines = [tasklines task(1)];
        plotNames{agents+t} = strcat("Task ",num2str(t));
    end
        %     title({"Green Agent is " + states_t(j,1) + "-ing; Purple Agent is ",...
%         states_t(j,2) + "-ing;  Orange Agent is " + states_t(j,3) + "-ing",...
%         "Yellow Agent is " + states_t(j,4) + "-ing"})
    %colormap(ax(1),cm)
    
    % extract the handles that require legend entries
    hleglines = [heglines tasklines path(1)];
    plotNames{agents+size(plt.task_t{1,j},1)+1} = "Planned Path";
    % create the legend
%     hleg = legend(hleglines,'Agent 1','Agent 2','Agent 3','Agent 4', 'Task 1','Task 2','Planned Path',...
%                     'Location','southoutside','NumColumns',4);
    hleg = legend(hleglines,plotNames,...
                    'Location','southoutside','NumColumns',5);
% 
    hleg.ItemTokenSize = [8,8];
    
    xlabel('$x$ (m)','interpreter','latex')
    xlim([0 length(plt.xDim)-1])
    xticks(0:50:length(plt.xDim))
    ylabel('$y$ (m)','interpreter','latex')
    ylim([0 length(plt.yDim)-1])
    yticks(0:50:length(plt.yDim))
    box on
    hold off
    set(gca,'LooseInset',get(gca,'TightInset'),'FontSize',12);
    subplot(1,3,3)
    hold on
%     axis square
    M0 = squeeze(plt.M_t(:,:,j));
    %Office_full
    tmp = plt.M_t(:,:,end);
    total_expl_denom = sum(sum(tmp<0.45 | tmp>0.55));
    expl_perc(j) = sum(sum(abs(M0-0.5)>0.45 &abs(M0-0.5)<0.55))/total_expl_denom*100;%/833*100;%;%(length(M0(:)))*100;
    taskComp(:,j) = plt.taskComp_t{1,j};

    pltExp = plot(1:j,expl_perc(1:j),'Color',[77,175,74]./255,'linewidth',2);
    heglines2 = pltExp(1);
    plotNames2{1} = "Explored";
    for t = 1:size(taskComp,1)
        pltTskRem = plot(1:j,taskComp(t,1:j),'Color',taskColors(t,:),'linewidth',2);
        heglines2 = [heglines2 pltTskRem(1)];
        if size(taskComp,1)>1
            plotNames2{1+t} = strcat("Task ",num2str(t)," Remaining");
        else
            plotNames2{1+t} = "Task Remaining";
        end
    end
    
%     plotlgd = legend("Explored","Task 1 Remaining","Task 2 Remaining","Location","southoutside","NumColumns",1);
    plotlgd = legend(heglines2,plotNames2,"Location","southoutside","NumColumns",1);
    plotlgd.ItemTokenSize = [8,8];
    xlabel("Time (steps)",'interpreter','latex')
    ylabel("Percent ($\%$)",'interpreter','latex')
    xlim([0 j+5])
    xticks('auto')
    ylim([0 103])
    yticks([0:20:103])
    grid on
    hold off
    box on
    set(gcf, 'color', 'white'); 
%     set(gca,'LooseInset',get(gca,'TightInset'),'FontSize',12);
    %bar(taskComp_t(j))
    F(j) = getframe(gcf);
    cla
    j
end

if plt.gif == "yes"
    testtime = datestr(now,'mm-dd-yyyy HH-MM-SS');
    video_filename = sprintf('%s-%s.avi', [plt.filename,testtime]);
    % % % create the video writer with 30 fps
      writerObj = VideoWriter(video_filename);
      writerObj.FrameRate = 30;
    % set the seconds per image
    % open the video writer
    open(writerObj);
    % write the frames to the video
    for i= 2:length(F)
        % convert the image to a frame
        frame = F(i);
        writeVideo(writerObj, frame);
    end
    % close the writer object
    close(writerObj);
end
%}

end