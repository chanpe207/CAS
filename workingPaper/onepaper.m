%creating a simple piece of paper in workspace
close all;
clear all;
clc
% 4.1 and 4.2: Define the DH Parameters to create the Kinematic model
L1 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi])
model = SerialLink([L1],'name','myModel')                     % Generate the model
linkIndex=0;
%%
[ faceData, vertexData, plyData{linkIndex + 1} ] = plyread(['whiteEnvelope',num2str(linkIndex),'.ply'],'tri'); % #ok<AGROW>
    model.faces{linkIndex + 1} = faceData;
    model.points{linkIndex + 1} = vertexData;
%%
model.base =  model.base * trotx(pi/2) * troty(pi/2) * transl([0,0,1]);   %transl matrix [y,z,x]

%%
workspace = [-4 4 -4 4 -4 4];                                       % Set the size of the workspace when drawing the robot

scale = 0.5;

q = zeros(1,1);                                                     % Create a vector of initial joint angles

model.plot(q,'workspace',workspace,'scale',scale);                  % Plot the robot
camlight
% 4.3 Manually play around with the robot
model.teach;                                                        % Open a menu to move the robot manually

% 4.4 Get the current joint angles based on the position in the model
q = model.getpos();  

% 4.5 Get the joint limits
model.qlim

model.delay = 0;


%%

    handles = findobj('Tag',  model.name);
    h = get(handles,'UserData');

        h.link(linkIndex+1).Children.FaceVertexCData = [plyData{linkIndex+1}.vertex.red ...
            , plyData{linkIndex+1}.vertex.green ...
            , plyData{linkIndex+1}.vertex.blue]/255;
        h.link(linkIndex+1).Children.FaceColor = 'interp';
%         disp(ME_1);

%%
%%
%%
%%
%%
%%
%%
%%
%%
%%
%%
%%
%%
%%
%%
%%
% % % COMPILING ALL VERSIONS
close all;
set(0,'DefaultFigureWindowStyle','dock')
workspace = [-3 3 -3 3 -3 3];
pause(0.001);
name = ['Linur3_',datestr(now,'yyyymmddTHHMMSSFFF')];
L1 = Link([pi 0 0 pi/2 1]); 
L1.qlim = [-0.8 0];
L1.offset = 0;
model = SerialLink([L1],'name',name);
model.base =  model.base * trotx(pi/2) * troty(pi/2) * transl([0,0,1]);   %transl matrix [y,z,x]
%% PlotAndColourRobot

% Given a robot index, add the glyphs (vertices and faces) and
% colour them in if data is available
for linkIndex = 0: model.n
    if  linkIndex ==  model.n
        [ faceData, vertexData, plyData{linkIndex + 1} ] = plyread(['whiteEnvelope',num2str(linkIndex),'.ply'],'tri'); % #ok<AGROW>
        model.faces{linkIndex + 1} = faceData;
        model.points{linkIndex + 1} = vertexData;
    else
    end

end

% Display Robot
model.plot3d(zeros(1, model.n),'noarrow','workspace', workspace);
if isempty(findobj(get(gca,'Children'),'Type','Light'))
    camlight
end
model.delay = 0;

q = model.getpos()%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Try to correctly colour the arm (if colours are in ply file data)
for linkIndex = 0: model.n
    
    if  linkIndex ==  model.n
        handles = findobj('Tag',  model.name);
        h = get(handles,'UserData');
        try
            h.link(linkIndex+1).Children.FaceVertexCData = [plyData{linkIndex+1}.vertex.red ...
                , plyData{linkIndex+1}.vertex.green ...
                , plyData{linkIndex+1}.vertex.blue]/255;
            h.link(linkIndex+1).Children.FaceColor = 'interp';
        catch ME_1
            disp(ME_1);
            continue;
        end

    else
    end
    
end
%^^ALL ABOVE IS NOT GONNA CHANGE^^
% % input('CONTINUE TO GRIPPER - BRICKS')
