function SprayPaintUR3(paperCorners)
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

%% Translate paper corners away from paper by x distance
distanceFromPaper = 0.2;
goalTopLeft = paperCorners(1).*transl(distanceFromPaper,0,0);
goalTopRight = paperCorners(2).*transl(distanceFromPaper,0,0);
goalBottomLeft = paperCorners(3).*transl(distanceFromPaper,0,0);
goalBottomRight = paperCorners(4).*transl(distanceFromPaper,0,0);


end

