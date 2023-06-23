load('plt_bubbles_main_smol.mat')
agents = size(plt.x_t,1);
GridPoints = [plt.X(:),plt.Y(:)];
for p = 2:plt.steps-1
    M0 = squeeze(plt.M_t(:,:,p));
    pos = squeeze(plt.x_t(:,:,p));
    last_pos = squeeze(plt.x_t(:,:,p-1));
    next_pos = squeeze(plt.x_t(:,:,p+1));
    [close,c_idx] = sort(sqrt((pos(:,2)-last_pos(:,2)).^2 + (pos(:,1)-last_pos(:,1)).^2));
    for k = 2:agents
        imp = c_idx(k);
        conflicts = ismember(pos(c_idx(1:k-1),:),pos(imp,:),'rows');
        if any(conflicts)
            [pts,D] = rangesearch(GridPoints,pos(imp,:),2,'Distance','euclidean');
            [c_r,c_c] = ind2sub(size(M0),pts{1});
            tmp_cands = setdiff([c_c',c_r'],pos(imp,:),'rows');
            D = D{1}(2:end);
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
        plt.x_t(imp,:,p) = pos(imp,:);
    end
end
f = figure;
% set(f, 'MenuBar', 'none');


cm = cbrewer('qual','Accent',agents);
blk = 64/255;
cm_unk = cm;

% axis tight manual % this ensures that getframe() returns a consistent size

for j = 1:plt.steps-1
%     subplot(1,2,1);
    heglines = [];
    hold on
    axis tight;
    h = pcolor(plt.X-1,plt.Y-1,squeeze(plt.M_t(:,:,j))); colormap(flipud(bone));
    set(h,'EdgeColor','none')
    for k = 1:agents
        ag_plot = plot(plt.x_t(k,1,j)-0.5,plt.x_t(k,2,j)-0.5,'Marker','o','MarkerFaceColor',cm(k,:),'MarkerEdgeColor',[64,64,64]./255,'Color',cm(k,:),'Linestyle', 'none');
        plot(plt.waypt_t(k,1,j)-0.5,plt.waypt_t(k,2,j)-0.5,'r*')
        path = plot(plt.pthObj_t{k,j}(:,1)-0.5,plt.pthObj_t{k,j}(:,2)-0.5,'-r');
        heglines = [heglines ag_plot(1)];
    end
    task = plot(plt.task_t{j}(:,1),plt.task_t{j}(:,2),'Marker','d','MarkerFaceColor',[55,126,184]./255,'Color',[55,126,184]./255,'MarkerEdgeColor','k','Linestyle', 'none','LineWidth',1.25);
%     title({"Green Agent is " + states_t(j,1) + "-ing; Purple Agent is ",...
%         states_t(j,2) + "-ing;  Orange Agent is " + states_t(j,3) + "-ing",...
%         "Yellow Agent is " + states_t(j,4) + "-ing"})
    %colormap(ax(1),cm)
    
    % extract the handles that require legend entries
    hleglines = [heglines task(1) path(1)];
    % create the legend
    hleg = legend(hleglines,'Agent 1','Agent 2','Agent 3','Task','Planned Path',...
                    'Location','southoutside','NumColumns',5);
    hleg.ItemTokenSize = [8,8];
    
    xlabel('$x$ (m)','interpreter','latex')
    xlim([0 length(plt.xDim)-1])
    xticks(0:20:length(plt.xDim))
    ylabel('$y$ (m)','interpreter','latex')
    ylim([0 length(plt.yDim)-1])
    yticks(0:20:length(plt.yDim))
    box on
    
    for t = 1:size(plt.taskLocs,1)
        rectangle('Position',[plt.taskLocs(t,1)+2 plt.taskLocs(t,2)+2 10 3],'FaceColor','white','Curvature',0.2)
        if j>1
            if plt.taskComp_t{j}(t)-plt.taskComp_t{j-1}(t)<0 && plt.taskComp_t{j}(t)>0
                rectangle('Position',[plt.taskLocs(t,1)+2 plt.taskLocs(t,2)+2 10*plt.taskComp_t{j}(t)/100 3],...
                    'FaceColor','red','Curvature',0.2,'EdgeColor','none')
            elseif plt.taskComp_t{j}(t)<0
                rectangle('Position',[plt.taskLocs(t,1)+2 plt.taskLocs(t,2)+2 0.01 3],...
                    'FaceColor','green','Curvature',0.2,'EdgeColor','none')
            else
                rectangle('Position',[plt.taskLocs(t,1)+2 plt.taskLocs(t,2)+2 10*plt.taskComp_t{j}(t)/100 3],...
                    'FaceColor','green','Curvature',0.2,'EdgeColor','none')
            end
        else
            rectangle('Position',[plt.taskLocs(t,1)+2 plt.taskLocs(t,2)+2 10*plt.taskComp_t{j}(t)/100 3],...
                'FaceColor','green','Curvature',0.2,'EdgeColor','none')
        end
    end
    hold off

    set(gcf, 'color', 'white'); 
    set(gca,'LooseInset',get(gca,'TightInset'));
    %bar(taskComp_t(j))
    F(j) = getframe(gcf);
    cla
    j
end

% if plt.gif == "yes"
%     testtime = datestr(now,'mm-dd-yyyy HH-MM-SS');
%     video_filename = sprintf('%s-%s', [plt.filename,testtime]);
%     % % % create the video writer with 30 fps
%       writerObj = VideoWriter(video_filename,'MPEG-4');
%       writerObj.FrameRate = 30;
%     % set the seconds per image
%     % open the video writer
%     open(writerObj);
%     % write the frames to the video
%     for i= 2:length(F)
%         % convert the image to a frame
%         frame = F(i);
%         writeVideo(writerObj, frame);
%     end
%     % close the writer object
%     close(writerObj);
% end
% %}
% 
