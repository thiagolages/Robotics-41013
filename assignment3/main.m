% Robotics 41013: Assessment Task #4 – Final Report
% Thiago Lages Rocha
% Student ID: 13357191

%%
clf; clear all; close all;
% dock figures
set(0, 'DefaultFigureWindowStyle', 'docked');


% The Utils class (created by me) will take care of a few handy functions
u = Utils();

%% Task 1: Use the Puma 560 from the Robotics toolbox
% and Visual Servoing over the model in Drum.ply with
% images/graphs and code inline in the report, which
% prove successful completion:
% - mount a simulated camera on the end-effector
% pointing out the z-axis of the last joint (3)




% robot
xmin = -1; xmax = +1.5;
ymin = -1; ymax = +1;
zmin = +0; zmax = +2;
q_zero = [0 0 0 0 0 0];
q0 = deg2rad([0, 45, 135, 0, 0, 0])';

q0_alternative = [1.6441,    1.2732,   -3.8720,   -0.1117,   -0.8029,   -2.6557]';

% create robot
mdl_puma560;

% plot robot
p560.base = transl(0,0,1);
%p560.plot(q_zero, 'workspace', [xmin xmax ymin ymax zmin zmax],...
%        'floorlevel',0, 'tile1color', [0.5 1 0.5], 'tile2color', [1 1 1], ...
%        'jointdiam', 4, 'base'); 

%p560.plot(q0', 'workspace', [xmin xmax ymin ymax zmin zmax], 'floorlevel',0, ...
%    'jointdiam', 3.5);
p560.plot(q0_alternative', 'workspace', [xmin xmax ymin ymax zmin zmax], 'floorlevel',0, ...
    'jointdiam', 3.5);
pause(0.5);
%axis equal;
axis([xmin, xmax, ymin, ymax, zmin, zmax]);

hold on;
cam = u.createCamera();
%T0 = p560.fkine(p560.getpos);       % get eef position
T0 = p560.fkine(q0_alternative);
cam.T = T0;                         % mount camera on eef

% plot camera
hold on;

cam.plot_camera('Tcam',T0, 'label','scale',0.15);
% for i=1:45
%     T1 = T0*troty((pi/2)*i/45);
%     cam.plot_camera('Tcam',T1, 'label','scale',0.15); %plot the actual camera in 3D space
%     pause(0.1);
% end

% To access the camera handle, just use cam.handle

% plot drum
[f,v,data,drum_h] = u.plotPLY('Drum.ply', transl(0.05,-0.5,0));

% - manually place at least 4 targets on the drum in Matlab (4)

% subtracting 0.2 in x after drum translation (after extracting the
% targets coordinates from the drum (image in report))
P1 = [0.4530, 0.1495, 0.5976] - [0.2,0,0]; 
P2 = [0.6422, 0.1495, 0.5976] - [0.2,0,0];
P3 = [0.6422, 0.0254, 0.5976] - [0.2,0,0];
P4 = [0.4530, 0.0254, 0.5976] - [0.2,0,0];

points = [P1; P2; P3; P4]';

points_h = plot3(points(1,1), points(2,1), points(3,1), 'r.', 'MarkerSize', 18);
points_h = plot3(points(1,2), points(2,2), points(3,2), 'g.', 'MarkerSize', 18);
points_h = plot3(points(1,3), points(2,3), points(3,3), 'b.', 'MarkerSize', 18);
points_h = plot3(points(1,4), points(2,4), points(3,4), 'm.', 'MarkerSize', 18);

% - find an initial pose where the circled area and targets can be seen (3)

m_points = mean(points');
%T1 = transl(0.3, 0.1, 0.9)*trotx(pi);
T1 = transl(m_points(1), m_points(2), 0.9)*trotx(pi);

%q1 = p560.ikine(T1, q0);
q1 = p560.ikcon(T1, q0);
q1 = p560.ikcon(T1, q0_alternative);
%pause(5);
p560.plot(q1);
T2 = p560.fkine(p560.getpos);
cam.T = T2;
cam.plot_camera('Tcam',T2, 'label','scale',0.15); 

cam.plot(points(:,1), 'r.', 'MarkerSize',18);
cam.hold(true);
cam.plot(points(:,2), 'g.', 'MarkerSize',18);
cam.hold(true);
cam.plot(points(:,3), 'b.', 'MarkerSize',18);
cam.hold(true);
cam.plot(points(:,4), 'm.', 'MarkerSize',18);
cam.hold(true);

%cam_plot = cam.plot(v', 'r.');
%e = T2-T1;

% - visually servo to a closer pose where only the white plate is visible (7);


% set the points to be at the edges of the camera plane

% pStar = [CAMERA_EDGES]



% - then inspect the 4 corners of the front window (the desired corner for inspection should be placed in the
% centre of the image plane). Note that when moving to and between targets you must only use Visual
% Servoing techniques not ikine/ikcon (8)

pStar = [cam.pp]'; % where the point should be in the image (center) = principal point

deltaT = 1/10;

lambda = 0.6;
depth = mean (points(3,:));

vel_p = [];
uv_p  = [];
history = [];
color = ["RED", "GREEN", "BLUE", "MAGENTA"];

no_plot = true; % decide whether to animate robot or not
curr_q = p560.getpos';

% loop for all points
for i=1:size(points, 2)
    % loop of the visual servoing
    ksteps = 0;
    
    while true
            
            ksteps = ksteps + 1;
            c = color(i);
            disp(strcat("Centering point #",num2str(i), ", ",c, " color to [",num2str(cam.pp(1)),",",num2str(cam.pp(2)),"]"));
            % compute the view of the camera
            uv = cam.plot(points(:,i)); % getting the coordinate
            disp(strcat("Current position = [",num2str(uv(1)),",",num2str(uv(2)),"]"));
            % actually plotting
            cam.plot(points(:,1), 'r.', 'MarkerSize',18);
            cam.hold(true);
            cam.plot(points(:,2), 'g.', 'MarkerSize',18);
            cam.hold(true);
            cam.plot(points(:,3), 'b.', 'MarkerSize',18);
            cam.hold(true);
            cam.plot(points(:,4), 'm.', 'MarkerSize',18);
            cam.hold(true);

            % compute image plane error as a column
            e = pStar-uv;   % feature error
            e = e(:);

            Zest = [];

            % compute the Jacobian
            if isempty(depth)
                % exact depth from simulation (not possible in practice)
                pt = homtrans(inv(Tcam), P);
                J = cam.visjac_p(uv, pt(3,:) );
            elseif ~isempty(Zest)
                J = cam.visjac_p(uv, Zest);
            else
                J = cam.visjac_p(uv, depth );
            end

            % compute the velocity of camera in camera frame
            try
                v = lambda * pinv(J) * e;
            catch
                status = -1;
                return
            end
            %fprintf('v: %.3f %.3f %.3f %.3f %.3f %.3f\n', v);

            %compute robot's Jacobian and inverse
            J2 = p560.jacobn(curr_q);
            Jinv = pinv(J2);
            % get joint velocities
            qp = Jinv*v;


             %Maximum angular velocity cannot exceed 180 degrees/s
             ind=find(qp>pi);
             if ~isempty(ind)
                 qp(ind)=pi;
             end
             ind=find(qp<-pi);
             if ~isempty(ind)
                 qp(ind)=-pi;
             end

            %Update joints 
            q = curr_q + deltaT*qp;
            if no_plot ~= true
                p560.animate(q');
            end
            %Get camera location
            Tc = p560.fkine(q');
            cam.T = Tc;

            drawnow()
            p560.animate(q');
            % update the history variables
            hist.uv = uv(:);
            vel = v;
            hist.vel = vel;
            hist.e = e;
            hist.en = norm(e);
            hist.jcond = cond(J);
            hist.Tcam = Tc;
            hist.vel_p = vel;
            hist.uv_p = uv;
            hist.qp = qp;
            hist.q = q;

            history = [history hist];

             %pause(1/fps);
            % stop if we are not moving the arm on a significant velocity
            if abs(sum(v)) <= 1e-4 || (ksteps > 200)
                break;
            end

            %update current joint position
            curr_q = q;
            % clear camera figure
            cam.clf;
            
     end %loop finishes
     disp("Done ! Resuming in 3s...");
     disp(strcat("Current error: [",num2str(e(1)'),",",num2str(e(2)'),"] (pixels)"));
     
     if no_plot ~= true
        pause(3);
     end
     p560.animate(q');
end

%% Task 2: Dynamic Torque
% calculations for grit blasting the Drum task with a Puma560
% from the Robotics toolbox.
%  In Matlab determine the tool transform offset to attach the
% blasting nozzle shown in the figure to the robot (4)

% discussion on how I got to this is on the report
tool_T = transl(0,0,0.20692)*trotx(+pi/4);
p560.tool = tool_T;

%  Either using a pose from Task 1 or using ikcon/ikine determine
% a starting pose that points the nozzle at one corner of the white window 
% on the drum (2)

d = 0.3; % desired distance from eef to point (in Z)
T = transl(P1(1), P1(2),P1(3) + d)* trotx(pi); % Z poiting down
tr_h = trplot(T);
%q = p560.ikine(T, q_zero);
q = p560.ikcon(T, q_zero);
p560.plot(q);
p560.fkine(q)
cam.h_visualize.Visible = 'off'; % hide camera

% Code the following two scenarios to move the arm with the 2kg tool (i.e. payload) in a straight line
% (blast stream pointing straight downwards) with a blasting reaction force (straight upwards)
% equivalent to 200N, between any two corners:
%  slow enough so none of the joints’ maximum torques are exceeded during motion (10)

% Use RMRC to create a straight line trajectory
pointA = P1; pointB = P2;
totalTime = 2.5; deltaT = 0.02;
q_guess = q;
[qMatrix, positionError, angleError] = u.solveAndPlotRMRC(p560, pointA, pointB, q_guess, totalTime, deltaT);
%%
% I'm considering the center of mass is exactly where the eef coordinate
% frame is, since ive calculated a point that lies within the eef, and this
% might actually be a good point to consider it (can change later if
% necessary)
% since the tool will always be pointing down, and the blast will produce a
% force straight up, we can consider a total payload 'always pushing up',
% since the blasting will be done all the time. So instead of having a 2kg
% mass pushing the arm down, well have a (200N - 2kg*9.81m/s^2) pushing up

mass = 2;
grav = 9.81;
tool_force = 200;
total_force = mass*grav - tool_force; % will be negative (means it points up)
total_payload = total_force/grav;     % to find the corresponding mass and use the .payload() function, we need to divide by the gravity

p560.payload(total_payload,[0,0,0]);

time = 10;                                                                  % Total time to execute the motion
dt = 1/50;  

dynamics = Dynamics();
[tau, qdd, qd, new_qMatrix] = dynamics.solveAndPlot(p560, qMatrix, time, dt);

%  fast enough to overload the arm so it cannot achieve the task (14)
%  For both for full marks, you should:
%  allow the full 6DOF to move whilst ensuring you obey joint limits.
%  include a separate plot of the angles / velocities / accelerations / torques on each joint in each motion.
%  include the code inline in the report.
%  Don’t include collision checking/avoidance or extra safety features.

