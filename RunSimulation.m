clear;
clf;

sp = SprayPaintingBots;
paperPose = eye(4,4);
paperPose = paperPose*transl(-0.2,0,1.1);
paperCornersAll = sp.GetPaperCorners(paperPose);
[auboi3Robot, ur3Robot, paperModel] = sp.PlotSprayPaintEnvironment;
sp.MovetoPaperAuboi3Sim(auboi3Robot, paperPose, paperModel);
[nextJointState_123456, movementDuration] = sp.SprayPaintUR3Sim(ur3Robot, paperCornersAll, 0);
sp.PutDownPaperAuboi3Sim(auboi3Robot, paperModel);