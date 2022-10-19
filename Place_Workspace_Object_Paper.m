function [brick_tr_all, brick_h] = Place_Workspace_Object_Paper(x,y,z)
    %% Paper using Trisurf
    [f, v, data] = plyread('whiteEnvelope1.ply','tri');
    data.vertex.red = data.vertex.x;
    data.vertex.green = data.vertex.y;
    data.vertex.blue = data.vertex.z;
    vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
    % brick_coords = [-0.6200    0.4000    0.3440];
        h.surf = trisurf(f,v(:,1)+ x,v(:,2)+y, v(:,3)+z, 'FaceVertexCData', vertexColours, 'EdgeColor', 'interp');
        brick_h(1) = h.surf;
        brick_tr_all(:,:,1) = transl(x,y,z)*troty(pi);
end
