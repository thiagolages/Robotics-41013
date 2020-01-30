classdef UR3 < handle
    properties
        %> Robot model
        model;
        
        %> workspace
        workspace = [-1 1 -1 1 0 1];   
             
        pointCloud = zeros(1); % just to create an attribute for the class, we dont know the size yet
        
        %> If we have a tool model which will replace the final links model, combined ply file of the tool model and the final link models
        toolModelFilename = []; % Available are: 'DabPrintNozzleTool.ply';        
        toolParametersFilename = []; % Available are: 'DabPrintNozzleToolParameters.mat';        

        % Log purposes
        L = log4matlab('log_UR3.log');

        
    end
    
    methods%% Class for UR3 robot simulation
        function self = UR3(toolModelAndTCPFilenames, robot_name)
            if 0 < nargin
                if length(toolModelAndTCPFilenames) ~= 2
                    error('Please pass a cell with two strings, toolModelFilename and toolCenterPointFilename');
                end
                self.toolModelFilename = toolModelAndTCPFilenames{1};
                self.toolParametersFilename = toolModelAndTCPFilenames{2};
            end
            
            self.GetUR3Robot(robot_name);
            %self.PlotAndColourRobot();%robot,workspace);

            drawnow            
            % camzoom(2)
            % campos([6.9744    3.5061    1.8165]);

%             camzoom(4)
%             view([122,14]);
%             camzoom(8)
%             teach(self.model);
        end

        %% GetUR3Robot
        % Given a name (optional), create and return a UR3 robot model
        function GetUR3Robot(self, robot_name)
            pause(0.001);
            disp(strcat('robot_name = ',robot_name));
            if strcmp(robot_name,'')
                name = ['UR3_',datestr(now,'yyyymmddTHHMMSSFFF')];
            else
                name = robot_name;
            end
            
            L1 = Link('d', 0.1519, 'a',        0, 'alpha',  pi/2,'qlim',deg2rad([-180 180]), 'offset', 0);
            L2 = Link('d', 0     , 'a', -0.24365, 'alpha',     0,'qlim',deg2rad([-180   0]), 'offset', 0);
            L3 = Link('d', 0     , 'a', -0.21325, 'alpha',     0,'qlim',deg2rad([-180 180]), 'offset', 0);
            L4 = Link('d',0.11235, 'a',        0, 'alpha',  pi/2,'qlim',deg2rad([-180 180]), 'offset', 0);
            L5 = Link('d',0.08535, 'a',        0, 'alpha', -pi/2,'qlim',deg2rad([-180 180]), 'offset', 0);
            L6 = Link('d', 0.0819, 'a',        0, 'alpha',     0,'qlim',deg2rad([-180 180]), 'offset', 0);
            
            self.model = SerialLink([L1 L2 L3 L4 L5 L6],'name',name);
            
            logFileName = strcat('log/log_UR3_',self.model.name,'.log');
            self.L = log4matlab(logFileName);
            disp(strcat("Logging on file: ",logFileName));


        end
        
        %% Set base transform
        function SetBase(self, tr)
            self.model.base = tr;
        end
                
        
        
        %% Plot given a joint state
        function PlotWithColor(self, q)%robot,workspace)
            for linkIndex = 0:self.model.n
                path = strcat('../models/robot/UR3/UR10Link',num2str(linkIndex),'.ply');
                [ faceData, vertexData, plyData{linkIndex + 1} ] = plyread([path],'tri'); %#ok<AGROW>
                
                %disp(strcat("Reading ",path));
                
                self.model.faces{linkIndex + 1} = faceData;
                self.model.points{linkIndex + 1} = vertexData;
            end

            if ~isempty(self.toolModelFilename)
                [ faceData, vertexData, plyData{self.model.n + 1} ] = plyread(self.toolModelFilename,'tri'); 
                self.model.faces{self.model.n + 1} = faceData;
                self.model.points{self.model.n + 1} = vertexData;
                toolParameters = load(self.toolParametersFilename);
                self.model.tool = toolParameters.tool;
                self.model.qlim = toolParameters.qlim;
                warning('Please check the joint limits. They may be unsafe')
            end
            % Display robot
            self.model.plot3d(q,'noarrow','workspace',self.workspace);
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end  
            self.model.delay = 0;

            % Try to correctly colour the arm (if colours are in ply file data)
            for linkIndex = 0:self.model.n
                
                %disp("inside color");
                
                handles = findobj('Tag', self.model.name);
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
        
        
        
        %% PlotAndColourRobot
        % Given a robot index, add the glyphs (vertices and faces) and
        % colour them in if data is available 
        function PlotAndColourRobot(self)%robot,workspace)
            for linkIndex = 0:self.model.n
                path = strcat('../models/robot/UR3/3-final-ply/link',num2str(linkIndex),'.ply');
                %path = strcat('../models/robot/UR3/3-final-ply/link2.ply');
                [ faceData, vertexData, plyData{linkIndex + 1} ] = plyread([path],'tri'); %#ok<AGROW>
                
                disp(strcat("Reading ",path));
                
                self.model.faces{linkIndex + 1} = faceData;
                self.model.points{linkIndex + 1} = vertexData;
            end

            if ~isempty(self.toolModelFilename)
                [ faceData, vertexData, plyData{self.model.n + 1} ] = plyread(self.toolModelFilename,'tri'); 
                self.model.faces{self.model.n + 1} = faceData;
                self.model.points{self.model.n + 1} = vertexData;
                toolParameters = load(self.toolParametersFilename);
                self.model.tool = toolParameters.tool;
                self.model.qlim = toolParameters.qlim;
                warning('Please check the joint limits. They may be unsafe')
            end
            % Display robot
            self.model.plot3d(zeros(1,self.model.n),'noarrow');%,'workspace',self.workspace);
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end  
            self.model.delay = 0;

            % Try to correctly colour the arm (if colours are in ply file data)
            for linkIndex = 0:self.model.n
                
                %disp("inside color");
                
                handles = findobj('Tag', self.model.name);
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
        
        %% Plot World
        function PlotWorld(self)
            figure(1);
            path = strcat('../models/world/world_color.ply');
            [f,v,data] = plyread(path,'tri');

            % Get vertex count
            worldVertexCount = size(v,1);

            % Move center point to origin
            midPoint = sum(v)/worldVertexCount;
            worldVerts = v - repmat(midPoint,worldVertexCount,1);

            % Create a transform to describe the location (at the origin, since it's centered
            worldPose = eye(4)*transl(0,0.6,0);

            % Scale the colours to be 0-to-1 (they are originally 0-to-255
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

            % Then plot the trisurf
            worldMesh_h = trisurf(f,worldVerts(:,1),worldVerts(:,2), worldVerts(:,3) ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
            
            updatedPoints = [worldPose * [worldVerts,ones(worldVertexCount,1)]']';  

            % Now update the Vertices
            worldMesh_h.Vertices = updatedPoints(:,1:3);
            
%                 for ax = 180:-1:0; %#ok<UNRCH>
%                     %for by = [[30:-3:0],[0:3:30]];
%                         by = 30;
%                         view(ax,by);
%                         axis equal;
%                         drawnow();
%                         pause(0.01);
%                     %end;
%                 end
            
            
%             for i = 1:1000
%                 % Random rotate about Z
%                 randRotateTR = makehgtform('zrotate',(rand-0.5)* 10 * pi/180);
% 
%                 % Move forward then random rotation
%                 monkeyPose = monkeyPose * forwardTR * randRotateTR;
% 
%                 % Transform the vertices
%                 updatedPoints = [monkeyPose * [monkeyVerts,ones(monkeyVertexCount,1)]']';
% 
%                 % Update the mesh vertices in the patch handle
%                 monkeyMesh_h.Vertices = updatedPoints(:,1:3);
%                 drawnow();   
%             end

            
            
            
            
            
%             for linkIndex = 0:self.model.n
%                 path = strcat('../models/world/world_color.ply');
%                 [ faceData, vertexData, plyData{linkIndex + 1} ] = plyread([path],'tri'); %#ok<AGROW>
%                 
%                 disp(strcat("Reading ",path));
%                 
%                 self.model.faces{linkIndex + 1} = faceData;
%                 self.model.points{linkIndex + 1} = vertexData;
%             end
% 
%             if ~isempty(self.toolModelFilename)
%                 [ faceData, vertexData, plyData{self.model.n + 1} ] = plyread(self.toolModelFilename,'tri'); 
%                 self.model.faces{self.model.n + 1} = faceData;
%                 self.model.points{self.model.n + 1} = vertexData;
%                 toolParameters = load(self.toolParametersFilename);
%                 self.model.tool = toolParameters.tool;
%                 self.model.qlim = toolParameters.qlim;
%                 warning('Please check the joint limits. They may be unsafe')
%             end
%             % Display robot
%             %self.model.plot3d(zeros(1,self.model.n),'noarrow','workspace',self.workspace);
%             if isempty(findobj(get(gca,'Children'),'Type','Light'))
%                 camlight
%             end  
%             self.model.delay = 0;
% 
%             % Try to correctly colour the arm (if colours are in ply file data)
%             for linkIndex = 0:self.model.n
%                 
%                 %disp("inside color");
%                 
%                 handles = findobj('Tag', self.model.name);
%                 h = get(handles,'UserData');
%                 try 
%                     h.link(linkIndex+1).Children.FaceVertexCData = [plyData{linkIndex+1}.vertex.red ...
%                                                                   , plyData{linkIndex+1}.vertex.green ...
%                                                                   , plyData{linkIndex+1}.vertex.blue]/255;
%                     h.link(linkIndex+1).Children.FaceColor = 'interp';
%                 catch ME_1
%                     disp(ME_1);
%                     continue;
%                 end
%             end
        end
        %%
        %% Plot Parts
        function [partMesh_h, partVerts, partVertexCount, updatedPoints] = PlotParts(self,part_ID, tr)
            path = strcat('../models/parts/3-final-ply/');
            parts = ["HousingTop_painted.ply","PCB_painted.ply","HousingBottom_painted.ply",];
            
            part = parts(part_ID);
            
            [f,v,data] = plyread(strcat(path,part),'tri');

            % Get vertex count
            partVertexCount = size(v,1);

            % Move center point to origin
            midPoint = sum(v)/partVertexCount;
            partVerts = v - repmat(midPoint,partVertexCount,1);

            % Create a transform to describe the location (at the origin, since it's centered
            partPose = eye(4);

            % Scale the colours to be 0-to-1 (they are originally 0-to-255
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

            % Then plot the trisurf
            partMesh_h = trisurf(f,partVerts(:,1),partVerts(:,2), partVerts(:,3) ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');

            % Move the pose forward and a slight and random rotation

            updatedPoints = [tr * [partVerts,ones(partVertexCount,1)]']';  

            % Now update the Vertices
            partMesh_h.Vertices = updatedPoints(:,1:3);
            %drawnow();
        end
        
        
        
        %% STATUS
        function PlotWithStatus(self,qArray, each)
            figure(1);
            count = 1;
            for i=1:size(qArray,1)
                
                self.model.plot(qArray(i,:));
                
                if mod(count,each) == 0
                    currPosition = transl(self.model.fkine(qArray(i,:)))';
                    message = strcat('DEBUG: ',self.model.name,' current position = ',num2str(currPosition));
                    self.L.mlog = {self.L.DEBUG,'UR3',message};
                    disp(message);
                end
                count = count + 1;
            end
            tf = transl(self.model.fkine(self.model.getpos))';
            message = strcat('DEBUG: Final position of ',self.model.name,' = ',num2str(tf));
            self.L.mlog = {self.L.DEBUG,'UR3',message};
            disp(message);
        end
        
        
        
        %% VOLUME
        function [h_] = PlotVolume(self, createNew, baseSecondRobot, color)
                       
            % first we get the mean point in between the robots
            % this point represents the point on a plane
            base1 = transl(self.model.base);
            base2 = transl(baseSecondRobot);
            pointOnPlane = ((base1 + base2)/2)'; % transpose to get a row
            normalOnPlane = ((base1 - base2)/norm(base1-base2))';
            
            stepRads = deg2rad(30);
            qlim = self.model.qlim;
            
            if createNew == 1 % only if pointCloud has not been create yet

                pointCloudeSize = prod(floor((qlim(1:5,2)-qlim(1:5,1))/stepRads + 1));
                self.pointCloud = zeros(pointCloudeSize,3);
                disp(num2str(size(self.pointCloud)));
                counter = 1;
                tic
                for q1 = qlim(1,1):stepRads:qlim(1,2)
                    for q2 = qlim(2,1):stepRads:qlim(2,2)
                        for q3 = qlim(3,1):stepRads:qlim(3,2)
                            for q4 = qlim(4,1):stepRads*2:qlim(4,2)
                                for q5 = qlim(5,1):stepRads*2:qlim(5,2)
                                    % Q6 actually doesnt affect the x,y,z, only the orientation 
                                    %for q6 = qlim(6,1):stepRads:qlim(6,2)
                                        q6 = 0;
                                        q = [q1,q2,q3,q4,q5,q6];
                                        tr = self.model.fkine(q);  
                                        
                                        if self.isSafe(tr, pointOnPlane, normalOnPlane)
                                            self.pointCloud(counter,:) = tr(1:3,4)';
                                            counter = counter + 1; 
                                        end
                                        
                                        if mod(counter/pointCloudeSize * 100,1) == 0
                                            display(['After ',num2str(toc),' seconds, completed ',num2str(counter/pointCloudeSize * 100),'% of poses']);
                                        end
                                    %end
                                end
                            end
                        end
                    end
                end
            end
            %figure;
            %self.model.plot([0 0 0 0 0 0]);
            hold on;
            if color == "red"
                color = [255 0 0]/255;
            else if color == "blue"
                    color = [0 0 255]/255;
                end
            end
            figure(1);
            h_ = plot3(self.pointCloud(:,1),self.pointCloud(:,2),self.pointCloud(:,3),'r.','Color',color);
            drawnow();
            hold on;
        end
        
    %% 
    function [safe] = isSafe (self, tr, pointOnPlane, normalOnPlane)
        pointOfInterest = transl(tr)';
        u = pointOfInterest - pointOnPlane;
        if dot(u,normalOnPlane) >= 0
            safe = true;
        else
            safe = false;
        end
    end
         
        
    end
end