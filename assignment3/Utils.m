classdef Utils < handle
    properties
           
    end
    
    methods(Static)
        function self = Utils()
            % do nothing
        end
        
        function [f,v,data,ply_handle] = plotPLY(filepath, tr)
            [f,v,data] = plyread(filepath,'tri');

            % Scale the colours to be 0-to-1 (they are originally 0-to-255
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

            count = size(v,1); % get number of vertices
            v = [v, ones(count, 1)]'; % appending a column of 1s to allow tr multiplication
            v = [tr * v]'; % make the transform
            v = v(:,1:3);

            % Then plot the trisurf
            hold on;
            ply_handle = trisurf(f,v(:,1),v(:,2), v(:,3) ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
        end
        
        function [cam] = createCamera()
            % need to 'hold on' before function call
            res_x = 1000; % to get a nice principal point
            res_y = 1000; % to get a nice principal point
            focal_length = 0.08;
            pixel_size = 10e-5;
            % camera_fps = 25;

            cam = CentralCamera('focal',focal_length , 'pixel', pixel_size, ...
            'resolution', [res_x res_y], 'centre', round([res_x/2 res_y/2]), 'name', 'myCamera');  
        end
        
        function [] = adjust()
            % adjust settings
            camlight;
            axis equal;
            view(3);
            hold on;
        end
        
        function [qMatrix, positionError, angleError] = solveAndPlotRMRC(p560, pointA, pointB, q0, totalTime, deltaT)
            
            t = totalTime;      % Total time (s)
            %deltaT = 0.02;      % Control frequency
            steps = t/deltaT;   % No. of steps for simulation
            epsilon = 0.1;      % Threshold value for manipulability/Damped Least Squares
            %W = diag([1 1 1 0.1 0.1 0.1]);    % Weighting matrix for the velocity vector
            W = eye(6);
            
            m = zeros(steps,1);             % Array for Measure of Manipulability
            qMatrix = zeros(steps,6);       % Array for joint anglesR
            qdot = zeros(steps,6);          % Array for joint velocities
            %theta = zeros(3,steps);         % Array for roll-pitch-yaw angles
            xyzTraj = zeros(3,steps);             % Array for x-y-z trajectory
            positionError = zeros(3,steps); % For plotting trajectory error
            angleError = zeros(3,steps);    % For plotting trajectory error

            % 1.3) Set up trajectory, initial pose
            s = lspb(0,1,steps);                % Trapezoidal trajectory scalar
            d = 0.3; % desired distance from eef to point (in Z)
            x0 = pointA(1);  x1 = pointB(1);
            y0 = pointA(2);  y1 = pointB(2);
            z0 = pointA(3) + d;  z1 = pointB(3) + d;
            for i=1:steps
                xyzTraj(1,i) = (1-s(i))*x0 + s(i)*x1; % Points in x
                xyzTraj(2,i) = (1-s(i))*y0 + s(i)*y1; % Points in y
                xyzTraj(3,i) = (1-s(i))*z0 + s(i)*z1; % Points in z
                % not defining rotations here. will define on next
                % transform
            end
            
            % rotation means Z pointing down
            T = transl(xyzTraj(1,1), xyzTraj(2,1), xyzTraj(3,1))* trotx(pi);          % Create transformation of first point and angle
            %q0 = initial_guess;                                                            % Initial guess for joint angles
            qMatrix(1,:) = p560.ikcon(T,q0);                                            % Solve joint angles to achieve first waypoint
            p560.plot(qMatrix(1,:));
            % 1.4) Track the trajectory with RMRC
            for i = 1:steps-1
                T = p560.fkine(qMatrix(i,:));                                           % Get forward transformation at current joint state
                deltaX = xyzTraj(:,i+1) - T(1:3,4);                                         	% Get position error from next waypoint
                Rd = rotx(pi);                          % Next rotation matrix is always the same (Z poiting down)
                Ra = T(1:3,1:3);                                                        % Current end-effector rotation matrix
                Rdot = (1/deltaT)*(Rd - Ra);                                                % Calculate rotation matrix error
                S = Rdot*Ra';                                                           % Skew symmetric!
                linear_velocity = (1/deltaT)*deltaX;
                angular_velocity = [S(3,2);S(1,3);S(2,1)];                              % Check the structure of Skew Symmetric matrix!!
                deltaTheta = tr2rpy(Rd*Ra');                                            % Convert rotation matrix to RPY angles
                xdot = W*[linear_velocity;angular_velocity];                          	% Calculate end-effector velocity to reach next waypoint.
                J = p560.jacob0(qMatrix(i,:));                 % Get Jacobian at current joint state
                m(i) = sqrt(det(J*J'));
                if m(i) < epsilon  % If manipulability is less than given threshold
                    lambda = (1 - m(i)/epsilon)*5E-2;
                else
                    lambda = 0;
                end
                invJ = inv(J'*J + lambda *eye(6))*J';                                   % DLS Inverse
                qdot(i,:) = (invJ*xdot)';                                                % Solve the RMRC equation (you may need to transpose the         vector)
                for j = 1:6                                                             % Loop through joints 1 to 6
                    if qMatrix(i,j) + deltaT*qdot(i,j) < p560.qlim(j,1)                     % If next joint angle is lower than joint limit...
                        qdot(i,j) = 0; % Stop the motor
                    elseif qMatrix(i,j) + deltaT*qdot(i,j) > p560.qlim(j,2)                 % If next joint angle is greater than joint limit ...
                        qdot(i,j) = 0; % Stop the motor
                    end
                end
                qMatrix(i+1,:) = qMatrix(i,:) + deltaT*qdot(i,:);                         	% Update next joint state based on joint velocities
                positionError(:,i) = xyzTraj(:,i+1) - T(1:3,4);                               % For plotting
                angleError(:,i) = deltaTheta;                                           % For plotting
            end
     
            % Plot the results
            figure(1)
            plot3(xyzTraj(1,:),xyzTraj(2,:),xyzTraj(3,:),'k.','LineWidth',1)
            p560.plot(qMatrix,'trail','r-')

            for i = 1:6
                figure(3)
                subplot(3,2,i)
                plot(qMatrix(:,i),'k','LineWidth',1)
                title(['Joint ', num2str(i)])
                ylabel('Angle (rad)')
                refline(0,p560.qlim(i,1));
                refline(0,p560.qlim(i,2));

                figure(4)
                subplot(3,2,i)
                plot(qdot(:,i),'k','LineWidth',1)
                title(['Joint ',num2str(i)]);
                ylabel('Velocity (rad/s)')
                refline(0,0)
            end

            figure(5)
            subplot(2,1,1)
            plot(positionError'*1000,'LineWidth',1)
            refline(0,0)
            xlabel('Step')
            ylabel('Position Error (mm)')
            legend('X-Axis','Y-Axis','Z-Axis')

            subplot(2,1,2)
            plot(angleError','LineWidth',1)
            refline(0,0)
            xlabel('Step')
            ylabel('Angle Error (rad)')
            legend('Roll','Pitch','Yaw')
            figure(6)
            plot(m,'k','LineWidth',1)
            refline(0,epsilon)
            title('Manipulability')
        end  
    end
end