function SprayPaintUR3(paperCoords)
%SPRAYPAINTUR3 Summary of this function goes here

%   create the UR3 at a point in the workspace
r = UR3;
% create variable for joints
q = zeros(1,r.model.n);

%% Move from zero position to position ready to spray

readyEEPosition = [-0.2; -0.2; 0.42];
readyEEAngles = [0 -pi/2 pi/2];
readyEEOrientation = eul2rotm(readyEEAngles);
steps = 20;
q1 = r.model.getpos();
T2 = cat(1,cat(2,readyEEOrientation,readyEEPosition),[0 0 0 1]);
q2 = r.model.ikcon(T2);

qMatrix = jtraj(q1,q2,steps);

for i = 1:steps
    r.model.animate(qMatrix(i,:));
    pause(0.01);
end

%% Translate paper corners away from paper by x distance
distanceFromPaper = 0.2;
goalPoints = paperCoords.*transl(distanceFromPaper,0,0);


end

