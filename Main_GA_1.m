
dt = 0.1;
PopSize = 10;
MaxGenerations = 5; 
Elite = ceil(0.35*PopSize);
Crossratio = 0.6;
Mutationrate = 0.01; 

global tr;
global tp;
global ty;
global vr;
global vp;
global vy;
global k;
k = 1;

figure; hAxes(1) = gca;
title("Roll")
tr = animatedline(hAxes(1),'Color','r');
vr = animatedline(hAxes(1),'Color','b');

figure; hAxes(2) = gca;
title("Pitch")
tp = animatedline(hAxes(2),'Color','r');
vp = animatedline(hAxes(2),'Color','b');

figure; hAxes(3) = gca;
title("Yaw")
ty = animatedline(hAxes(3),'Color','r');
vy = animatedline(hAxes(3),'Color','b');

clear gaoutfun
options = optimoptions(@ga,'PopulationSize',PopSize,'MaxGenerations',MaxGenerations,....
'EliteCount' ,Elite ,'CrossoverFcn', {@crossoverintermediate, Crossratio},'MutationFcn', {@mutationuniform, Mutationrate},'PlotFcn', {@gaplotbestf, @gaplotbestindiv},'OutputFcn',@gaoutfun);

lb = [0.01 0 0 0.01 0 0 0 0 0];
ub = [0.2 0.5 0.009 0.2 0.5 0.009 0.6 0.5 0.001];
options.InitialPopulationRange = [lb;ub];

global dialect
global gcsNode
global gcsPort
global uavClient
global paramValueSub
global Atti_1
global Atti_2


dialect = mavlinkdialect("common.xml");
gcsNode = mavlinkio(dialect);
gcsPort = 14540;
connect(gcsNode,"UDP", 'LocalPort', gcsPort);
uavClient = mavlinkclient(gcsNode,1,1);
paramValueSub = mavlinksub(gcsNode,uavClient,'PARAM_VALUE','BufferSize',10,'NewMessageFcn', @(~,msg)disp(msg.Payload));

Atti_1 = mavlinksub(gcsNode,uavClient,'ATTITUDE','NewMessageFcn', @(~,msg)disp(msg.Payload));
Atti_2 = mavlinksub(gcsNode,uavClient,'ATTITUDE_TARGET','NewMessageFcn', @(~,msg)disp(msg.Payload));

[x,fval,exitflag,output,population,scores] = ga(@(K)GA_2(K),9,-eye(9),zeros(9,1),[],[],lb,ub,[],options);

record   = gaoutfun();
obj = final_set(record,PopSize,MaxGenerations);
disconnect(gcsNode);
disconnect(gcsNode);
disconnect(gcsNode);
figure
plot(obj);
drawnow