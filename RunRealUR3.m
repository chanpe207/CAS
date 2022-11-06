clear;
clf;

% Create spray can "safety symbol" to simulation collision
spawnOn = 0; %spawn in spray can
objectCenter = [-0.4 -0.4 0.7]; % spray can location
radius = 0.3; % radius of collision sphere

sp = SprayPaintingBots; % declare and name class

% Paper spraying location
paperPose = eye(4,4);
paperPose = paperPose*transl(-0.2,0,1.1);
paperCornersAll = sp.GetPaperCorners(paperPose);

% Plot the simulation environment, safety symbol, and start GUI
[auboi3Robot, ur3Robot, paperModel] = sp.PlotSprayPaintEnvironment;
[objectCenter,radius] = sp.SpawnSafetySymbol(spawnOn,objectCenter,radius);
gui = guiTest;

%% Start Spray Painting
sp.MovetoPaperAuboi3Sim(auboi3Robot, paperPose, paperModel, gui, objectCenter, radius); 
[nextJointState_123456, movementDuration] = sp.SprayPaintUR3Sim(ur3Robot, paperCornersAll, 0, gui,objectCenter,radius);
sp.PutDownPaperAuboi3Sim(auboi3Robot, paperModel, gui,objectCenter,radius);

%% Run real UR3
close all;
clf;
sp.StartRealUR3