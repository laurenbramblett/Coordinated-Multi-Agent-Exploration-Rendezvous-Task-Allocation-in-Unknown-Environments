function makeVideoMultAgent(plt)
f = figure('Position',[100 100 1280/2 720/2]);
% f = figure();
set(f, 'MenuBar', 'none');

agents = size(plt.x_t,1);
cm = cbrewer('qual','Accent',agents);
blk = 64/255;
cm_unk = cm;

%  axis square; % this ensures that getframe() returns a consistent size
arr = 1:agents*3;
arr = arr(mod(arr,3)>0);
for j = 1:plt.steps-1
    subplot(agents,3,arr);
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
        if plt.taskComp_t{j}(t)>0
            task = plot(plt.task_t{1,j}(t,1)-0.5,plt.task_t{1,j}(t,2)-0.5,...
                'Marker','d','MarkerFaceColor',taskColors(t,:),'Color',...
                taskColors(t,:),'MarkerEdgeColor','k','Linestyle', 'none','LineWidth',1.25);
        else
            task = plot(NaN,NaN,...
                'Marker','d','MarkerFaceColor',taskColors(t,:),'Color',...
                taskColors(t,:),'MarkerEdgeColor','k','Linestyle', 'none','LineWidth',1.25);
        end
        tasklines = [tasklines task(1)];
        plotNames{agents+t} = strcat("Task ",num2str(t));
    end



    % extract the handles that require legend entries
    hleglines = [heglines tasklines path(1)];
    plotNames{agents+size(plt.task_t{1,j},1)+1} = "Planned Path";
    % create the legend
    hleg = legend(hleglines,plotNames,...
                    'Location','southoutside','NumColumns',5);
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
    for k = 1:agents
        subplot(agents,3,k*3)
        hold on
        mk = reshape(plt.Mk_t(:,k,j),size(plt.X));
        h = pcolor(plt.X-1,plt.Y-1,mk); colormap(flipud(bone)); clim([0,1])
        set(h,'EdgeColor','none')
        plot(plt.x_t(k,1,j)-0.5,plt.x_t(k,2,j)-0.5,'Marker','o','MarkerFaceColor',cm(k,:),'MarkerEdgeColor',[64,64,64]./255,'Color',cm(k,:),'Linestyle', 'none');
        plot(plt.waypt_t(k,1,j)-0.5,plt.waypt_t(k,2,j)-0.5,'r*')
        plot(plt.pthObj_t{k,j}(:,1)-0.5,plt.pthObj_t{k,j}(:,2)-0.5,'-r');
        taskColors = [55,126,184;
                  152,78,163]./255;
        for t = 1:size(plt.task_t{1,j},1)
            if plt.taskComp_t{j}(t)>0
                plot(plt.task_t{1,j}(t,1)-0.5,plt.task_t{1,j}(t,2)-0.5,...
                    'Marker','d','MarkerFaceColor',taskColors(t,:),'Color',...
                taskColors(t,:),'MarkerEdgeColor','k','Linestyle', 'none','LineWidth',1.25);
            end
        end
        xlabel('$x$ (m)','interpreter','latex')
        xlim([0 length(plt.xDim)-1])
        xticks(0:50:length(plt.xDim))
        ylabel('$y$ (m)','interpreter','latex')
        ylim([0 length(plt.yDim)-1])
        yticks(0:50:length(plt.yDim))
    end
%     set(gca,'LooseInset',get(gca,'TightInset'),'FontSize',12);
    %bar(taskComp_t(j))
    set(gcf, 'color', 'white');
    F(j) = getframe(gcf);
    cla; clf
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

end