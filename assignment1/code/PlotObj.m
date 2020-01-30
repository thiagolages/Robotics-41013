function [objMesh_h] = PlotObj(objPose, ID)%robot,workspace)
    
    switch ID
        case 1
            path = strcat('../models/parts/HousingTop.ply');
        case 2
            path = strcat('../models/parts/HousingBottom_painted.ply');
        case 3
            path = strcat('../models/parts/PCB.ply');
        otherwise
            disp("Insert a valid ID, from 1 to 3. (top, bottom, pcb).");
    end
    [ faceData, vertexData, data ] = plyread([path],'tri'); %#ok<AGROW>

    % Get vertex count
    objVertexCount = size(vertexData,1);

    % Move center point to origin
    midPoint = sum(vertexData)/objVertexCount;
    objVerts = vertexData - repmat(midPoint,objVertexCount,1);

    % Scale the colours to be 0-to-1 (they are originally 0-to-255
    vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

    % Then plot the trisurf
    objMesh_h = trisurf(faceData,objVerts(:,1),objVerts(:,2), objVerts(:,3) ...
    ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');

    updatedPoints = [objPose * [objVerts,ones(objVertexCount,1)]']';  
    objMesh_h.Vertices = updatedPoints(:,1:3);
end        