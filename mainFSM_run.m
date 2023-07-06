%run main
clc; clear all; close all
addpath('utils')
tests = 1;
pltCell = cell(tests,1);
for t = tests:tests %12 for three agent
%% Set seed
seed = t;
rng(seed)

%% Initialize map
% Initialize map
whichMap = "clutter"; %[mapY, playground, maze, complex, warehouse,clutter]
[obs, M0,x_scale,y_scale] = setMap(whichMap);

%% Video setup/ File Save
plt.gif = "no";
plt.filename = "plt_exploreOnlyWithRz3Agents";
plt.save = "yes";
plt.agentPerspectiveGraph = "yes";

%% Run main
runType = "FSM"; %[FSM = proposed approach, meetplan = always rendezvous, SR = always connected]
% Initial robot locations
% %Experiments
% x0 = [3 2];
% x1 = [5 2];
% x2 = [7 2];
%Sims
x0 = [3 4];
x1 = [2 4];
x2 = [3 3];
% x3 = [3 4];
% x4 = [3 4];
% x5 = [2 2];
% x6 = [2 3];
% x7 = [2 4];
% x8 = [3 4];
% x9 = [3 4];
% x10 = [2 2];
% x11 = [2 3];
% x12 = [2 4];
% x13 = [2 3];
% x14 = [2 4];
% x15 = [3 4];
% x16 = [3 4];
% x17 = [2 2];
% x18 = [2 3];
% x19 = [2 4];
x = [x0; x1; x2]; %x3; x4; x5];% x6; x7; x8; x9; x10; x11; x12; x13; x14; x15; x16; x17; x18; x19];%x3;x4];% x5; x6; x7]; %+100;

names = {'a', 'b', 'thresh','eV', 'varEps','r','taskGen','task','taskCompGlobal','plotInline'};
% Variable Explanations: 
% a = alpha; 
% b = beta; 
% thresh = rendezvous trigger; 
% eV = expected task length; 
% varEps (gamma in the paper) = risk tolerance for traversing unknown regions to rendezvous
% r = sensor radius
% taskGen - set to 1 if randomly generate task
% task = set taskGen to 0 and put coordinates for tasks here (nx2)
% taskCompGlobal = time to complete task (nx1)
% plotInline = ugly/slow inline plotting to debug
values = {1,1,20,1,1,5,0,[8 18; 10 10],[125; 2],0}; %Two tasks & two task lengths
% values = {1,1,150,200,1,10,0,[-10 -10],500,0}; %Gazebo
chars = cell2struct(values, names, 2);
[plt,totalTime] = robotsMain(plt,seed,obs,M0,x,runType,chars);
% save('plt_bubbles_main_smol.mat','plt')
%% Store
% fid = fopen('testing_3agents_v2.txt', 'a+');
% fprintf(fid, 'runType: %s, Seed: %d, Total Time: %d, Task Completion Time: %d, Exploration Completion Time: %d, Task Location: [%d %d]\n', runType, seed, totalTime, taskComplete,explorationComplete,plt.taskLocs);
% fclose(fid);
% pltCell{t} = plt;
end
%% Make Video
% load('plt_perspective_clutter_24Feb_DiffCent.mat')
if plt.gif == "yes"
    [plt] = makeVideo(plt,0);
end
if plt.save == "yes"
    save(plt.filename)
end
if plt.agentPerspectiveGraph == "yes"
    makeVideoMultAgent(plt);
end

