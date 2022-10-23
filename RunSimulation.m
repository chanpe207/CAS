clear;
clf;
%%
% 'SprayPaintingBots' funct redefined as 'sp'
sp = SprayPaintingBots;
% paper position
paperPose = transl(-0.2,0,1.1);
    % paperPose = eye(4,4);
    % paperPose = paperPose*transl(-0.2,0,1.1);

% location of all 4 paper corners
paperCornersAll = sp.GetPaperCorners(paperPose); %VALUES OF PAPER

%% ---%end of constants%---%


%% objects loaded into workspace
% AND COLLISION WITH ur3
[auboi3Robot, ur3Robot, paperModel] = sp.PlotSprayPaintEnvironment;
% sp.auboChecksCollision(auboi3Robot) %variable inputs are done in the functions script called

%%
auboCollisionCheck(auboi3Robot)

%%








% % MOVEMENT OF ROBOTS
%%
%% MOVE TO GRAB PAPER - AUBOi3
sp.MovetoPaperAuboi3Sim(auboi3Robot, paperPose, paperModel);

%% SPRAY PAPER (6 PTS) - UR3
[nextJointState_123456, movementDuration] = sp.SprayPaintUR3Sim(ur3Robot, paperCornersAll, 0);

%% PUTS DOWN PAPER - AUBOi3
sp.PutDownPaperAuboi3Sim(auboi3Robot, paperModel);