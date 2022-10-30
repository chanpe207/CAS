clear;
clf;

spawnOn = 0; %spawn in spray can to simulate collision
sp = SprayPaintingBots;
paperPose = eye(4,4);
paperPose = paperPose*transl(-0.2,0,1.1);
paperCornersAll = sp.GetPaperCorners(paperPose);
[auboi3Robot, ur3Robot, paperModel] = sp.PlotSprayPaintEnvironment;
[objectCenter,radius] = sp.SpawnSafetySymbol(spawnOn);
gui = guiTest;

%% Start Spray Painting
sp.MovetoPaperAuboi3Sim(auboi3Robot, paperPose, paperModel, gui); 
[nextJointState_123456, movementDuration] = sp.SprayPaintUR3Sim(ur3Robot, paperCornersAll, 0, gui);
sp.PutDownPaperAuboi3Sim(auboi3Robot, paperModel, gui);