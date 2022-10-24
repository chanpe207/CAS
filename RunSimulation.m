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
%preallocate gui variables:
gui.aubo_q1=0;
gui.aubo_q2=0;
gui.aubo_q3=0;
gui.aubo_q4=0;
gui.aubo_q5=0;
gui.aubo_q6=0;
gui.aubo_start1=1;
gui.aubo_start2=1;
gui.aubo_stop=0;
gui.aubo_x=0;
gui.aubo_y=0;
gui.aubo_z=0;

gui.ur_q1=0;
gui.ur_q2=0;
gui.ur_q3=0;
gui.ur_q4=0;
gui.ur_q5=0;
gui.ur_q6=0;
gui.ur3_start1=1;
gui.ur3_start2=1;
gui.ur3_stop=0;
gui.ur_x=0;
gui.ur_y=0;
gui.ur_z=0;

%% Start Spray Painting
sp.MovetoPaperAuboi3Sim(auboi3Robot, paperPose, paperModel, gui); 
[nextJointState_123456, movementDuration] = sp.SprayPaintUR3Sim(ur3Robot, paperCornersAll, 0, gui);
sp.PutDownPaperAuboi3Sim(auboi3Robot, paperModel, gui);