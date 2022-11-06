function PlotSprayPaintEnvironment
workspace = [4 -4 4 -4 3 0];

r = GetAuboi3();
PlotAndColourRobot(r, workspace);

hold on
mesh_environment = PlaceObject('decimated_enviroment.PLY');
vertices = get(mesh_environment,'Vertices');
transformedVertices = [vertices,ones(size(vertices,1),1)] * transl(-2,0,0)';
transformedVertices = transformedVertices * trotx(pi/2)';
set(mesh_environment,'Vertices',transformedVertices(:,1:3));
drawnow();
axis equal
end

