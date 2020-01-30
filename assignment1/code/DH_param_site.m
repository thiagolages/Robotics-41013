L1 = Link('d', 0.1519, 'a',        0, 'alpha',  pi/2);
L2 = Link('d', 0     , 'a', -0.24365, 'alpha',     0);
L3 = Link('d', 0     , 'a', -0.21325, 'alpha',     0);
L4 = Link('d',0.11235, 'a',        0, 'alpha',  pi/2);
L5 = Link('d',0.08535, 'a',        0, 'alpha', -pi/2);
L6 = Link('d', 0.0819, 'a',        0, 'alpha',     0);
robot = SerialLink([L1 L2 L3 L4 L5 L6]);
q0 = [0 0 0 0 0 0];
robot.plot(q0);
load('ur3_q.mat');

for i=1:1851
    if (mod(i,10) == 0)
        robot.plot(q(i,:));
    end
end
