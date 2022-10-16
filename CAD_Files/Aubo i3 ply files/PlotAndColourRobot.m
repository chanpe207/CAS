function PlotAndColourRobot(r, workspace)
    for linkIndex = 1:r.n
        [ faceData, vertexData, plyData{linkIndex+1} ] = plyread(['part_',num2str(linkIndex),'.PLY'],'tri'); %#ok<AGROW>

        r.faces{linkIndex+1} = faceData;
        r.points{linkIndex+1} = vertexData;
    end

    % Display robot
    hold on
    r.plot3d(zeros(1,r.n),'noarrow','workspace',workspace);
    if isempty(findobj(get(gca,'Children'),'Type','Light'))
        camlight
    end  
    r.delay = 0;

    % Try to correctly colour the arm (if colours are in ply file data)
    for linkIndex = 0:r.n
        handles = findobj('Tag', r.name);
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
end