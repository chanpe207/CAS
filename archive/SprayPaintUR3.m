function SprayPaintUR3(paperCornersAll, paperMoving)
%SPRAYPAINTUR3 Summary of this function goes here

%   create the UR3 at a point in the workspace
r = UR3;
% create variable for joints
q = zeros(1,r.model.n);

%% Move from zero position to position ready to spray

% readyEEPosition = [-0.2133; -0.1942; 0.4809];
% readyEEAngles = [pi 0 -pi/2];
% readyEEOrientation = eul2rotm(readyEEAngles);
% T2 = cat(1,cat(2,readyEEOrientation,readyEEPosition),[0 0 0 1]);
steps = 20;
q1 = r.model.getpos();
q2 = [0 -pi/2 pi/2 pi 0 0];

qMatrix = jtraj(q1,q2,steps);

for i = 1:steps
    r.model.animate(qMatrix(i,:));
    pause(0.01);
end

%% Begin when everything is ready

if paperMoving == 0
    %% Translate paper corners away from paper by x distance
    distanceFromPaper = 0.2;
    goalTopLeft = paperCornersAll(:,:,1)*transl(0,distanceFromPaper,0);
    goalTopRight = paperCornersAll(:,:,2)*transl(0,distanceFromPaper,0);
    goalBottomLeft = paperCornersAll(:,:,3)*transl(0,distanceFromPaper,0);
    goalBottomRight = paperCornersAll(:,:,4)*transl(0,distanceFromPaper,0);
    
    %% Make two more waypoints in the centre of the paper
    distx = goalTopRight(1,4)-goalTopLeft(1,4);
    disty = goalBottomRight(1,4)-goalBottomLeft(1,4);
    goalTopCentre = goalTopLeft*transl(distx/2,disty/2,0);
    goalBottomCentre = goalBottomLeft*transl(distx/2,disty/2,0);
    
    paperPoints = [goalTopLeft; goalBottomLeft; goalTopCentre; goalBottomCentre; goalTopRight; goalBottomRight];
    numPaperPoints = 6;
    
    for i = 1:numPaperPoints
        paperPointsAll(:,:,i) = paperPoints((i-1)*4+(1:4),1:4);
    end
    %% Spray the paper at all points (when paper stops moving)
    for j = 1:numPaperPoints
        q1 = ur3Robot.model.getpos();
        TR = paperPointsAll(:,:,j);
        q2 = ur3Robot.model.ikine(TR); % ,'q',[pi pi],'mask',[1 1 1 0 0 0])
        
        qMatrix = jtraj(q1,q2,steps);
        
        for i = 1:steps
            ur3Robot.model.animate(qMatrix(i,:));
            pause(0.1);
        end
     end
    
    %% Return to ready and wait for next paper
    q1 = r.model.getpos();
    q2 = [0 -pi/2 pi/2 pi 0 0];
    
    qMatrix = jtraj(q1,q2,steps);
    
    for i = 1:steps
        r.model.animate(qMatrix(i,:));
        pause(0.01);
    end
end
end

