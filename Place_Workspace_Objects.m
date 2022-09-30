function [num_bricks, brick_coords, brick_tr_all, brick_h] = Place_Workspace_Objects()
%PLACE_WORKSPACE_OBJECTS This function creates the workspace objects for
%the Dual Cobot UR3Linear work area
hold on;

%% Bricks

% for i = 1:6
%     mesh_brick_{i} = PlaceObject('HalfSizedRedGreenBrick.ply');
%     vertices = get(mesh_brick_{i},'Vertices');
%     
%     transformedVertices = [vertices,ones(size(vertices,1),1)] * transl((-0.38-i*0.08),0.5,0.344)'; %translates object
%     set(mesh_brick_{i},'Vertices',transformedVertices(:,1:3));
%     
%     % transformedVertices = [vertices,ones(size(vertices,1),1)] * trotx(pi/2)'; %rotates object
%     % set(mesh_brick_{1},'Vertices',transformedVertices(:,1:3));
%     drawnow();
%     brick_coords(i,:) = [(-0.38-i*0.08),0.5,0.344];
% end
%% Brick using Trisurf
[f, v, data] = plyread('HalfSizedRedGreenBrick.ply','tri');
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
num_bricks = 9;
brick_thickness = 0.03354;
% brick_coords = [   0.0800    0.4000    0.3440; ...
%                    -0.0600    0.4000    0.3440; ...
%                    -0.2000    0.4000    0.3440; ...
%                    -0.3400    0.4000    0.3440; ...
%                    -0.4800    0.4000    0.3440; ...
%                    -0.6200    0.4000    0.3440; ...
%                    -0.7600    0.4000    0.3440; ...
%                    -0.9000    0.4000    0.3440; ...
%                    -1.0400    0.4000    0.3440];
brick_coords = [   -0.6200    0.4000    0.3440+brick_thickness; ...
                        -0.4800    0.4000    0.3440+brick_thickness; ...
                        -0.3400    0.4000    0.3440+brick_thickness; ...
                        -0.3400    0.4000    0.3440; ...
                        -0.4800    0.4000    0.3440; ...
                        -0.6200    0.4000    0.3440; ...
                        -0.6200    0.4000    0.3440+(brick_thickness*2); ...
                        -0.4800    0.4000    0.3440+(brick_thickness*2); ...
                        -0.3400    0.4000    0.3440+(brick_thickness*2)];

for i = 1:num_bricks
    h.surf = trisurf(f,v(:,1)+ brick_coords(i,1),v(:,2)+brick_coords(i,2), v(:,3)+brick_coords(i,3), ...
        'FaceVertexCData', vertexColours, 'EdgeColor', 'interp');
    brick_h(i) = h.surf;
    brick_tr_all(:,:,i) = transl(brick_coords(i,1),brick_coords(i,2),brick_coords(i,3))*troty(pi);
end

%% Table
table_y = [0;0.76;0;0.76];
table_x = [-0.3;-0.3;-1.5;-1.5];
for i = 1:size(table_y)
        mesh_table_{i} = PlaceObject('Palette_garden_table_charlize.ply');
        vertices = get(mesh_table_{i},'Vertices');
               
    %     transformedVertices = [vertices,ones(size(vertices,1),1)] * trotx(pi/2)'; %rotates object
    %     set(mesh_table_{i},'Vertices',transformedVertices(:,1:3));
    
        transformedVertices = [vertices,ones(size(vertices,1),1)] * transl(table_x(i),table_y(i),0)'; %translates object
        set(mesh_table_{i},'Vertices',transformedVertices(:,1:3));
        drawnow();
end
%% Fences
fence_x = [-2.436;0   ;2.436;3.654 ;3.654;-3.654;-3.654;-2.436;0    ];
fence_y = [-1.5  ;-1.5;-1.5 ;-0.282;2.154;-0.282; 2.154;3.372 ;3.372];
for j = 1:size(fence_x)
    
    switch j
        case {1,2,3} %back fence wall
            mesh_fence_{j} = PlaceObject('Fence_x_charlize.ply');
            vertices = get(mesh_fence_{j},'Vertices');

            transformedVertices = [vertices,ones(size(vertices,1),1)] * transl(fence_x(j),fence_y(j),0)'; %translates object
            set(mesh_fence_{j},'Vertices',transformedVertices(:,1:3));
            drawnow();
%            a=1 %debugging
    
    
        case {4,5} %side fence wall
            mesh_fence_{j} = PlaceObject('Fence_yflipped_charlize.ply');
            vertices = get(mesh_fence_{j},'Vertices');
    
            transformedVertices = [vertices,ones(size(vertices,1),1)] * transl(fence_x(j),fence_y(j),0)'; %translates object
            set(mesh_fence_{j},'Vertices',transformedVertices(:,1:3));
            drawnow();
%             a=2 %debugging
    
        case {6,7} %side fence wall
            mesh_fence_{j} = PlaceObject('Fence_y_charlize.ply');
            vertices = get(mesh_fence_{j},'Vertices');
    
            transformedVertices = [vertices,ones(size(vertices,1),1)] * transl(fence_x(j),fence_y(j),0)'; %translates object
            set(mesh_fence_{j},'Vertices',transformedVertices(:,1:3));
            drawnow();
%             a=3 %debugging
        case {8,9}
            mesh_fence_{j} = PlaceObject('Fence_xflipped_charlize.ply');
            vertices = get(mesh_fence_{j},'Vertices');
    
            transformedVertices = [vertices,ones(size(vertices,1),1)] * transl(fence_x(j),fence_y(j),0)'; %translates object
            set(mesh_fence_{j},'Vertices',transformedVertices(:,1:3));
            drawnow();
%             a=4 %debugging
    end
end

end

