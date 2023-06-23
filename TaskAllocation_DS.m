%% Rendezvous robots POV
clc; clear all
addpath('utils')
mu = 2000;
tm = 90;
te = 100;
px = 60;
a = 0:0.01:1;
b = 1-a;
k = 100;
kl = k-1;
ka = 0:k-1; kb = k-ka-1;
[a1,ka1] = meshgrid(a,ka);

u = -4.*a1.*(mu-tm-te)./(ka1+1)+(1-a1).*(te-px).*(k-1-ka1);
a2 = a1(:);
ka2 = ka1(:);
u2 = u(:);

CT = cbrewer('qual','Dark2',3);

% f = figure();
% hold on
% scatter3(a2,ka2,u2)
% hold off;

subplot(1,2,2);
hold on
% CT = cbrewer('div','Spectral',20);
[xq,yq] = meshgrid(0:0.01:1, (0:0.01:kl));
vq = griddata(ka2./kl,a2,u2,yq./kl,xq);
h = pcolor(yq./kl,xq,vq); colormap(winter); axis square;
set(h,'EdgeColor','none');


[val,rbts] = max(u,[],1);
% F = scatteredInterpolant(a,rbts,val);
fs = interparc(20,a,(rbts-1)./kl,val,'spline');
plot(fs(:,2),fs(:,1),'-r','linewidth',2)
% plt = plot(ka,-u(:,1),ka,u(:,2),ka,sum(u,2),'LineWidth',2);
% colororder(CT)
% legend('Estimated time to complete task', 'Estimated entropy gain','Objective value')

xlabel('$k_c$','interpreter','latex','FontSize',14)
ylabel('$\alpha$','interpreter','latex','FontSize',14)
zlabel('Objective value','interpreter','latex')
xt = xticks;
% hold off
hold off
%% 3D
subplot(1,2,1);
hold on
grid on
sh = mesh(yq./kl,xq,vq); colormap(winter)
plot3(fs(:,2),fs(:,1),fs(:,3)+10,'r','linewidth',2);
xlabel('$k_c$','interpreter','latex','FontSize',14)
ylabel('$\alpha$','interpreter','latex','FontSize',14)
zlabel('$z$','interpreter','latex','FontSize',13)
hold off
h = rotate3d;
set(h,'ActionPreCallback',...
    'set(gcf,''windowbuttonmotionfcn'',@align_axislabel)')
set(h,'ActionPostCallback',...
    'set(gcf,''windowbuttonmotionfcn'','''')')



