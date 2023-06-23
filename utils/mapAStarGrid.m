function pthObj = mapAStarGrid(x0,target,M0)

M = M0>0.85;
M = M';
map = binaryOccupancyMap(M);
%contour(occupancyMatrix(map))
planner = plannerAStarGrid(map);
start = round(x0,0);
goal = round(target,0);
rng(100, 'twister') % repeatable result
[pthObj, solnInfo] = plan(planner,start,goal);
%pthObj = flip(pthObj,1);
end
% show(planner)
% hold on
% plot(start(2),start(1),'g*',goal(2),goal(1),'r*')
% hold off