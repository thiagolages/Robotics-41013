function PuttingSimulatedObjectsIntoTheEnvironment()

% This function use "plyread" function which is available here:
% http://au.mathworks.com/matlabcentral/fileexchange/5355-toolbox-graph/content/toolbox_graph/read_ply.m

clf
doCameraSpin = false;

% Although generally I usually try to use more descriptive variables, to make it easier to view here I will use 
% f = faceData;
% v = vertexData;

%% Load the cube created and painted in Blender
[f,v,data] = plyread('cube.ply','tri');

% Scale the colours to be 0-to-1 (they are originally 0-to-255
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

hold on;

% Plot the cubes at the corners of a 10m square room
for xOffset = [-5, 5]
    for yOffset = [-5, 5]
        % Then plot the trisurf with offset verticies
        trisurf(f,v(:,1)+ xOffset,v(:,2) + yOffset, v(:,3) ...
            ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
    end
end

% Turn on a light (only turn on 1, don't keep turning them on), and make axis equal
camlight;
axis equal;
view(3);
hold on;

keyboard
% clf

%% Load the table downloaded from http://tf3dm.com/3d-model/wooden-table-49763.html vertex colours added with Blender

[f,v,data] = plyread('table.ply','tri');

% Scale the colours to be 0-to-1 (they are originally 0-to-255
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

% Then plot the trisurf
tableMesh_h = trisurf(f,v(:,1),v(:,2), v(:,3) ...
    ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');

keyboard
% clf

%% Load R2D2 from % http://tf3dm.com/3d-model/puo-4029-44927.html coloured in Blender
[f,v,data] = plyread('R2D2.ply','tri');

% Scale the colours to be 0-to-1 (they are originally 0-to-255
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

% Then plot the trisurf
r2D2Mesh_h = trisurf(f,v(:,1),v(:,2), v(:,3) ...
    ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');


%% Flying monkey's head from blender
% After saving in blender then load the triangle mesh
[f,v,data] = plyread('monkey.ply','tri');

% Get vertex count
monkeyVertexCount = size(v,1);

% Move center point to origin
midPoint = sum(v)/monkeyVertexCount;
monkeyVerts = v - repmat(midPoint,monkeyVertexCount,1);

% Create a transform to describe the location (at the origin, since it's centered
monkeyPose = eye(4);

% Scale the colours to be 0-to-1 (they are originally 0-to-255
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

% Then plot the trisurf
monkeyMesh_h = trisurf(f,monkeyVerts(:,1),monkeyVerts(:,2), monkeyVerts(:,3) ...
    ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');


%% To spin the camera 
if doCameraSpin
    for ax = -40:5:40; %#ok<UNRCH>
        for by = [[30:-3:0],[0:3:30]];
            view(ax,by);
            drawnow();
            pause(0.01);
        end;
    end
end

%% Then move the object once using Robot Toolbox transforms and without replot
axis([-10,10,-10,10,-2,2]);

% Move forwards (facing in -y direction)
forwardTR = makehgtform('translate',[0,-0.01,0]);

% Random rotate about Z
randRotateTR = makehgtform('zrotate',(rand-0.5)* 10 * pi/180);

% Move the pose forward and a slight and random rotation
monkeyPose = monkeyPose * forwardTR * randRotateTR;
updatedPoints = [monkeyPose * [monkeyVerts,ones(monkeyVertexCount,1)]']';  

% Now update the Vertices
monkeyMesh_h.Vertices = updatedPoints(:,1:3);

%% Now move it many times
for i = 1:1000
    % Random rotate about Z
    randRotateTR = makehgtform('zrotate',(rand-0.5)* 10 * pi/180);

    % Move forward then random rotation
    monkeyPose = monkeyPose * forwardTR * randRotateTR;

    % Transform the vertices
    updatedPoints = [monkeyPose * [monkeyVerts,ones(monkeyVertexCount,1)]']';
    
    % Update the mesh vertices in the patch handle
    monkeyMesh_h.Vertices = updatedPoints(:,1:3);
    drawnow();   
end

keyboard


