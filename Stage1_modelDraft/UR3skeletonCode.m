close all;
set(0,'DefaultFigureWindowStyle','dock')
workspace = [-2 2 -2 2 -0.3 2];
surf([-1.8,-1.8;1.8,1.8],[-1.8,1.8;-1.8,1.8],[0.01,0.01;0.01,0.01],'CData',imread('launcheri.png'),'FaceColor','texturemap');
pause(0.001);

name = ['toolbox_robot_UR3',datestr(now,'yyyymmddTHHMMSSFFF')];
L1 = Link([pi 0 0 pi/2 1]);
L2 = Link('d',0.1519,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]), 'offset',0);
L3 = Link('d',0,'a',-0.24365,'alpha',0,'qlim', deg2rad([-90 90]), 'offset',-pi/2);
L4 = Link('d',0,'a',-0.21325,'alpha',0,'qlim', deg2rad([-170 170]), 'offset', 0);
L5 = Link('d',0.11235,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]),'offset', -pi/2);
L6 = Link('d',0.08535,'a',0,'alpha',-pi/2,'qlim',deg2rad([-360,360]), 'offset',0);
L7 = Link('d',0.0819,'a',0,'alpha',0,'qlim',deg2rad([-360,360]), 'offset', 0);
L1.qlim = [-0.8 0];
L1.offset = 0;

%GRIPPER LINKS...

%CREATET MODEL
model = SerialLink([L1 L2 L3 L4 L5 L6 L7],'name',name);

% ROTATE BASE OF MODEL TO CORRECT ORIENTATION
model.base =  model.base * trotx(pi/2) * troty(pi/2) * transl([0,0,1]);

%% MODEL VISUALISATION
for linkIndex = 0: model.n
%     if  useGripper && linkIndex ==  model.n
%         [ faceData, vertexData, plyData{linkIndex+1} ] = plyread(['LinUR5Link',num2str(linkIndex),'Gripper.ply'],'tri'); %#ok<AGROW>
%     else
%         [ faceData, vertexData, plyData{linkIndex+1} ] = plyread(['LinUR5Link',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>
% 
%     end
    [ faceData, vertexData, plyData{linkIndex + 1} ] = plyread(['UR3LINK',num2str(linkIndex),'.ply'],'tri'); % #ok<AGROW>
    model.faces{linkIndex + 1} = faceData;
    model.points{linkIndex + 1} = vertexData;
end

% Display Robot
model.plot3d(zeros(1, model.n),'noarrow','workspace', workspace);
if isempty(findobj(get(gca,'Children'),'Type','Light'))
    camlight
end
model.delay = 0;

%% COLOURING OF MODEL (IF COLOURS ARE IN PLY FILE DATA)
% Try to correctly colour the arm (if colours are in ply file data)
for linkIndex = 0: model.n
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
end

%%
q = model.getpos()
