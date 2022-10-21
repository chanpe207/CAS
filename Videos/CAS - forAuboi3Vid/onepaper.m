
function loadPaperModel = onepaper(x,y,z)
%creating a simple piece of paper in workspace
% close all;
% clear all;
% clc
set(0,'DefaultFigureWindowStyle','dock')
workspace = [-2 2 -2 2 -1 2];
pause(0.001);

name = ['objectPaper1_',datestr(now,'yyyymmddTHHMMSSFFF')];

L1 = Link([pi 0 0 pi/2 0]); 
L1.qlim = [-pi pi]; %L1.qlim = [-0.8 0];
L1.offset = 0;
loadPaperModel = SerialLink([L1],'name',name);

% end goal position (AKA. paper coordiates)
x = -0.605
y = 0.242
z = 0
% x_O = endEffectorPos(1,4) %- 0.1
% y_O = endEffectorPos(2,4)
% z_O = endEffectorPos(3,4) %- 0.1 %this one cause of suction VERTICALLY

% start of paper
startpaper = [-0.605, 0.242, 0]
% to end of paper
endpaper = [-4.4547e-17, 0.1475, 0.7820]



% EEdefaultPos = [endEffectorPos(1,4),endEffectorPos(2,4),endEffectorPos(3,4)]
loadPaperModel.base =  loadPaperModel.base * transl([x,y,z])   %transl matrix [y,z,x]
%  * trotx(pi/2) * troty(pi/2)

%% PlotAndColour
% Given a robot index, add the glyphs (vertices and faces) and
% colour them in if data is available
for linkIndex = 0: loadPaperModel.n
    if  linkIndex ==  loadPaperModel.n
        [ faceData, vertexData, plyData{linkIndex + 1} ] = plyread(['whiteEnvelope',num2str(linkIndex),'.ply'],'tri'); % #ok<AGROW>
        loadPaperModel.faces{linkIndex + 1} = faceData;
        loadPaperModel.points{linkIndex + 1} = vertexData;
    else
    end

end

% Display
loadPaperModel.plot3d(zeros(1, loadPaperModel.n),'noarrow','workspace', workspace);
if isempty(findobj(get(gca,'Children'),'Type','Light'))
    camlight
end
loadPaperModel.delay = 0;
%%
% Try to correctly colour the arm (if colours are in ply file data)
for linkIndex = 0: loadPaperModel.n
    
    if  linkIndex ==  loadPaperModel.n
        handles = findobj('Tag',  loadPaperModel.name);
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


%% NEW!! copying values from grabPaper for aubo grabbng paper
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
axis equal
axis([-1 1 -1 1 -0.5 2]);

% current starting pos (anywhere as of current position)
q1_hardcoded = r.getpos()
% to destination 
q2_hardcoded = T2

qMatrix = jtraj(q1_hardcoded,q2_hardcoded,steps)

%% iteration
%pick up paper
for i = 1:steps
    r.animate(qMatrix(i,:));
    drawnow()
end

%%
secondgetposatpaperlocation = r.getpos()
spraypaintpos = [0 0 0 0 0 0]

currentpos = secondgetposatpaperlocation
finalpos = [0 0 0 0 0 0]
qMatrix = jtraj(currentpos,finalpos,steps)


%%
for i = 1:steps
    
%    new!!    
    refq = r.fkine(qMatrix(i,:))
    
%     loadPaperModel.base = eye(4) * trotx(pi/100)% * troty(pi/(2*(51-i)))
    loadPaperModel.base =  transl([refq(1,4), refq(2,4)-0.19, refq(3,4)])% * trotx(pi/100);%because divided by steps = 50
    loadPaperModel.animate(0);

    for j = 0:pi/2
    loadPaperModel.base = loadPaperModel.base * trotx(j)
    loadPaperModel.animate(0);    
    end
    r.animate(qMatrix(i,:));
    drawnow()
    
%this one v is the last pose!    
%     loadPaperModel.base = transl([-4.4547e-17, 0.1475-0.19, 0.7820])*trotx(pi/2);loadPaperModel.animate(0);


    %wrong - if above works delete this v
%     loadPaperModel.base = loadPaperModel.base * trotx(deg2rad(4.5)) * transl([0.03025, -0.004725, 0.0391]);
%     loadPaperModel.animate(0);
    

    
%     loadPaperModel.base = transl([-4.4547e-17, 0.1475-0.19, 0.7820])*trotx(pi/2);loadPaperModel.animate(0);
%     each location along the traject ory (qmatrix)
%     becomeing transl
%     and having -02 added to it (relative length!)
    
    
end
%% works until here - date:221021 time:2:38am





























end

%% moving model
% % % % % step = 0.1;
% % % % % for i=1:step:3
% % % % %     model.base =  model.base * transl([0,0.1,0.1]);
% % % % %     model.animate(0);
% % % % %     pause(0.1)
% % % % % end
% % % % % % 
% % % % % % for i = [[0:2:90],[89:-2:0]]
% % % % % %     try delete(R_h); end
% % % % % % 
% % % % % %     if rotationAxis == 1 % X axis rotation
% % % % % %         R = rotx(i * pi/180);
% % % % % %     elseif rotationAxis == 2 % Y axis rotation
% % % % % %         R = roty(i * pi/180);
% % % % % %     else % rotationAxis ==3 % Z axis rotation
% % % % % %         R = rotz(i * pi/180);
% % % % % %     end
% % % % % % 
% % % % % %     R_h = trplot(R,'frame','1', 'rgb','arrow');
% % % % % %     drawnow();
% % % % % %     pause(0.01);
% % % % % % end
% % % % % % pause
