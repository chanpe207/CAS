%% seriallink OF BRICKS
name = ['Linur3_BRICKS_',datestr(now,'yyyymmddTHHMMSSFFF')];

L1= Link('d',LOCbrick1(1,3),'a',LOCbrick1(1,1),'alpha',0,'qlim',deg2rad([-360,360]), 'offset', 0);    %
L2= Link('d',LOCbrick2(1,3),'a',LOCbrick2(1,1),'alpha',0,'qlim',deg2rad([-360,360]), 'offset', 0);    %
L3= Link('d',LOCbrick3(1,3),'a',LOCbrick3(1,1),'alpha',0,'qlim',deg2rad([-360,360]), 'offset', 0);    %
L4= Link('d',LOCbrick4(1,3),'a',LOCbrick4(1,1),'alpha',0,'qlim',deg2rad([-360,360]), 'offset', 0);    %
L5= Link('d',LOCbrick5(1,3),'a',LOCbrick5(1,1),'alpha',0,'qlim',deg2rad([-360,360]), 'offset', 0);    %
L6= Link('d',LOCbrick6(1,3),'a',LOCbrick6(1,1),'alpha',0,'qlim',deg2rad([-360,360]), 'offset', 0);    %
L7= Link('d',LOCbrick7(1,3),'a',LOCbrick7(1,1),'alpha',0,'qlim',deg2rad([-360,360]), 'offset', 0);    %
L8= Link('d',LOCbrick8(1,3),'a',LOCbrick8(1,1),'alpha',0,'qlim',deg2rad([-360,360]), 'offset', 0);    %
L9= Link('d',LOCbrick9(1,3),'a',LOCbrick9(1,1),'alpha',0,'qlim',deg2rad([-360,360]), 'offset', 0);    %

Paper1 = SerialLink(L1,'name',name);
Paper2 = SerialLink(L2,'name',name);
Paper3 = SerialLink(L3,'name',name);
Paper4 = SerialLink(L4,'name',name);
Paper5 = SerialLink(L5,'name',name);
Paper6 = SerialLink(L6,'name',name);
Paper7 = SerialLink(L7,'name',name);
Paper8 = SerialLink(L8,'name',name);
Paper9 = SerialLink(L9,'name',name);

for linkIndex = 1: cbrmodel.n
    [ faceData, vertexData, plyData{linkIndex} ] = plyread(['HalfSizedRedGreenBrick',num2str(linkIndex),'.ply'],'tri'); % #ok<AGROW>
    cbrmodel.faces{linkIndex+1} = faceData;
    cbrmodel.points{linkIndex+1} = vertexData;
end

% Display brick
cbrmodel.plot3d(zeros(1, cbrmodel.n),'noarrow','workspace', workspace);
if isempty(findobj(get(gca,'Children'),'Type','Light'))
    camlight
end
cbrmodel.delay = 0;

q_brick = cbrmodel.getpos()%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Try to correctly colour the arm (if colours are in ply file data)
for linkIndex = 0: cbrmodel.n
    handles = findobj('Tag',  cbrmodel.name);
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







