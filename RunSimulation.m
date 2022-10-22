clear;
clf;

sp = SprayPaintingBots;
paperPose = eye(4,4);
paperPose = paperPose*transl(-0.3035,0,0.857);
paperCornersAll = sp.GetPaperCorners(paperPose);
[auboi3Robot, ur3Robot] = sp.PlotSprayPaintEnvironment;
sp.MovePaperAuboi3Sim(auboi3Robot);
[nextJointState_123456, movementDuration] = sp.SprayPaintUR3Sim(ur3Robot, paperCornersAll, 0);
sp.PutDownPaperAuboi3Sim(auboi3Robot);