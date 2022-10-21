function grabPaper(r, brick_goal_coords)%function grabPaper(r, q, brick_goal_coords, brick_tr_all, brick_h)%function grabPaper(r, brick_tr_all, q, brick_h)%function grabPaper(brick_coords, r, gripper1, gripper2, brick_tr_all, q, brick_h)
%% variables
[f, v, data] = plyread('whiteEnvelope1.ply','tri');
data.vertex.red = data.vertex.x;
data.vertex.green = data.vertex.y;
data.vertex.blue = data.vertex.z;
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

%%% Place Object
% h.surf = trisurf(f,v(:,1)+ x,v(:,2)+y, v(:,3)+z, 'FaceVertexCData', vertexColours, 'EdgeColor', 'interp');
% brick_h = h.surf;

%% redefine friday/final day promo video update
% end goal joint angles [hard coded based on r.teach]
q1 = deg2rad(0)
q2 = deg2rad(72)
q3 = deg2rad(6.8)
q4 = deg2rad(12)
q5 = deg2rad(-90)
q6 = deg2rad(90)
T2 = [q1 q2 q3 q4 q5 q6]
% end goal position (AKA. paper coordiates)
x = -0.605
y = 0.242
z = 0
steps = 50
%%
brick_h = trisurf(f,v(:,1)+ x,v(:,3)+y, v(:,2)+z, 'FaceVertexCData', vertexColours, 'EdgeColor', 'interp');
%%
axis equal
axis([-1 1 -1 1 -0.5 2]);
%%% joint of robots
% q1 = r.getpos(); %robot/start pos
% T2 = transl([x y z]) * troty(pi) %coord of paper
%belowNabove is 2 different things
% % T2 = transl([x y z+0.14])
% % q2 = r.ikcon(T2) %paper/goal pos
% --

% current starting pos (anywhere as of current position)
q1_hardcoded = r.getpos()
% to destination 
q2_hardcoded = T2

qMatrix = jtraj(q1_hardcoded,q2_hardcoded,steps)

% Ta = r.fkine(r.getpos())
% Tb = transl([x y z])
% qMatrix = ctraj(Ta,Tb,steps)

%% iteration
%pick up paper
for i = 1:steps
    r.animate(qMatrix(i,:));
    drawnow()
    pause(0.2)
end
%%
secondgetposatpaperlocation = r.getpos()
spraypaintpos = [0 0 0 0 0 0]

currentpos = secondgetposatpaperlocation
finalpos = [0 0 0 0 0 0]
qMatrix = jtraj(currentpos,finalpos,steps)


%%
for i = 1:steps
    r.animate(qMatrix(i,:));
    drawnow()
    pause(0.2)
    
    
    
    
    
    
end
%% works until here - date:221021 time:2:38am
%%







g_distance = -0.14;    %const
for i = 1:steps

    % Object location - stuck under end effector (NOTE: qMatrix is EE location)
    current_brick_tr = r.fkine(qMatrix(i,:))*transl(0,0,-g_distance);
    delete(brick_h);
    brick_h = trisurf(f,v(:,1)+ current_brick_tr(1,4),v(:,2)+current_brick_tr(2,4), v(:,3)+current_brick_tr(3,4), 'FaceVertexCData', vertexColours, 'EdgeColor', 'interp');

    r.animate(qMatrix(i,:));
%     TrajTransform = r.ikcon(qMatrix(:,:,i))
%     r.animate(TrajTransform)
    drawnow()
    pause(0.2)
end
%%





% problem is the below??
%         brick_tr_all(:,:,1) = transl(x,y,z)*troty(pi);
brick_tr_all(:,:,1) = transl(x,y,z);


    %values
    brick_goaltr_all(:,:,1) = transl(brick_goal_coords(1,1),brick_goal_coords(1,2),brick_goal_coords(1,3))*troty(pi); %(1,3) - 1 was count of obj
    g_distance = -0.14;    %const
    brick_number_index = 1 %const

  
%%


%% end of my edit




































%% redundant (redefined later on in code)
% % % % Get pose of end effector
% % % EE_pose = r.fkine(r.getpos());
% % % current_brick_tr = brick_tr_all(:,:,brick_number_index)*transl(0,0,g_distance);
% % % 
% % % %if gripper is closed loop
% % % current_brick_tr = EE_pose*transl(0,0,-g_distance);

% % % %% NOTE of brick
% % % current brick tr
% % % was its position * up abit
% % % then
% % % was the end effector pose * up abit

%% determine the end goal for each brick
current_brick_goaltr = brick_goaltr_all(:,:,brick_number_index)*transl(0,0,g_distance);
%% Move UR3 to place brick at goal
steps = 100;
q1 = r.getpos();
T2 = current_brick_goaltr;
q2 = r.ikcon(T2);
qMatrix = jtraj(q1,q2,steps);





delete(brick_h);





%simulate end effector as it approaches end goal
for i = 1:steps
%     r.animate(qMatrix(i,:));
    r.plot(qMatrix(i,:));
    % determine the location of the end effector after each step
    EE_pose = r.fkine(r.getpos());
    % brick location animated to end effector
    current_brick_tr = EE_pose*transl(0,0,-g_distance);
    delete(brick_h);
    brick_h = trisurf(f,v(:,1)+ current_brick_tr(1,4),v(:,2)+current_brick_tr(2,4), v(:,3)+current_brick_tr(3,4), ...
    'FaceVertexCData', vertexColours, 'EdgeColor', 'interp');    
%     h.surf = trisurf(f,v(:,1)+ current_brick_tr(1,4),v(:,2)+current_brick_tr(2,4), v(:,3)+current_brick_tr(3,4), ...
%         'FaceVertexCData', vertexColours, 'EdgeColor', 'interp'); 
%     brick_h = h.surf;
    drawnow();
    display('loop_',num2str(i))
end

%determine if end effector has reached end goal
EE_pose = r.fkine(r.getpos());
end
