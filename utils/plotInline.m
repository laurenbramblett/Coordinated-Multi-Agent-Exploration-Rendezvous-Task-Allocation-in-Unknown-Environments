function plotInline(plt,i,Mtot,gridSize)
    figure(1);
    hold on
    pcolor(plt.X-1,plt.Y-1,reshape(Mtot,gridSize)); axis square; shading interp; colormap(flipud(bone))
    
    for k = 1:plt.agents
        plot(plt.x_t(k,1,i)-1,plt.x_t(k,2,i)-1,'bo',...
            squeeze(plt.x_t(k,1,1:i))-1,squeeze(plt.x_t(k,2,1:i))-1,'-.k',...
            plt.waypt_t(k,1,i)-1,plt.waypt_t(k,2,i)-1,'r*',...
            plt.pthObj_t{k,i}(:,1)-1,plt.pthObj_t{k,i}(:,2)-1,'-r',...
            plt.taskLocs(:,1)-1,plt.taskLocs(:,2)-1,'yd')
    end
    hold off
    drawnow
    if mod(i,50) == 0
        cla
    end

end