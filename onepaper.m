
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

% model.base =  model.base * trotx(pi/2) * troty(pi/2) * transl([0,0,1]);   %transl matrix [y,z,x]
loadPaperModel.base =loadPaperModel.base * transl([x,y,z]);

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
