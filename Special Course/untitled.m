clear all
close all
clc

%%

robot = robotics.RigidBodyTree('DataFormat','column','MaxNumBodies',3);
l1 = 1;
l2 = 1;

% Rigid body
body = robotics.RigidBody('link1');
% Create a joint and assign it to the rigid body

jnt1 = robotics.Joint('jnt1','revolute');

% Define the home position property of the joint.
jnt1.HomePosition = pi/2;

% Set the joint-to-parent transform using a homogeneuous transform
% 'tform'. Use the trvec2tform to convert translation vector to
% homogeneous transformation.
tform =trvec2tform([0, 0, 0])* eul2tform([deg2rad(-60),0,0]);
setFixedTransform(jnt1,tform);
jnt1.JointAxis = [0 0 1];
jnt1.PositionLimits = [-pi pi]
body.Joint = jnt1;
addBody(robot,body,'base')


body2 = robotics.RigidBody('link2');
jnt2 = robotics.Joint('jnt2','revolute');
jnt2.HomePosition = pi/2
tform2 = trvec2tform([l1,0,0]);
setFixedTransform(jnt2,tform2);
jnt2.JointAxis = [0 0 1];
jnt2.PositionLimits = [-pi pi/2]
body2.Joint = jnt2;
addBody(robot, body2, 'link1');

body3 = robotics.RigidBody('tool');
jnt3 = robotics.Joint('fix1','fixed');
tform3 = trvec2tform([l2,0,0]);
setFixedTransform(jnt3,tform3);
body3.Joint = jnt3;
addBody(robot, body3, 'link2');


show(robot)
showdetails(robot)
%%
% Test 1
t = (0:0.2:10)'; % Time
count = length(t);
center = [0.3 0.1 0];
radius = 0.15;
theta = t*(2*pi/t(end));
points = center + radius*[cos(theta) sin(theta) zeros(size(theta))];


% Inverse Kinematics Solution
% Use an InverseKinematics object to find a solution of robotic 
% configurations that achieve the given end-effector positions 
% along the trajectory. 
% Pre-allocate configuration solutions as a matrix qs.


q0 = homeConfiguration(robot);
ndof = length(q0);
qs = zeros(count, ndof);


% Create the inverse kinematics solver. Because the xy Cartesian points are
% the only important factors of the end-effector pose for this workflow,
% specify non-zero weight for the fourth and fifth elements of the weight
% vector. all other elements are set to zero.

ik = robotics.InverseKinematics('RigidBodyTree', robot);
weights = [0, 0, 0, 1, 1, 0];
endEffector = 'tool';



qInitial = q0; % Use home configuration as the initial guess
for i = 1:count
    % Solve for the configuration satisfying the desired end effector
    % position
    point = points(i,:)
    qSol = ik(endEffector,trvec2tform(point),weights,qInitial);
    % Store the configuration
    qs(i,:) = qSol;
    % Start from prior solution
    qInitial = qSol;
end

figure
show(robot,qs(1,:)');
view(2)
ax = gca;
ax.Projection = 'orthographic';
hold on
plot(points(:,1),points(:,2),'k')
% axis([-0.1 0.7 -0.3 0.5])


framesPerSecond = 15;
r = robotics.Rate(framesPerSecond);
for i = 1:count
    show(robot,qs(i,:)','PreservePlot',false);
    drawnow
    waitfor(r);
end
%%

q=linspace(0,pi/2,300)
pose = []
for i =1:length(q)
    tformi = trvec2tform([l1,0,0])*eul2tform([q(i),0,0]);
    pose{i} = tformi;
end

p = []
for i =1:length(q)
    T = pose{i}
    p = [hom2cart(T) p];
end

area(p(1,:),p(2,:))
% grid on

% Test 2
t = (0:0.2:10)'; % Time
count = length(t);
center = [0.3 0.1 0];
radius = 0.15;
theta = t*(2*pi/t(end));
points = center + radius*[cos(theta) sin(theta) zeros(size(theta))];


% Inverse Kinematics Solution
% Use an InverseKinematics object to find a solution of robotic 
% configurations that achieve the given end-effector positions 
% along the trajectory. 
% Pre-allocate configuration solutions as a matrix qs.


q0 = homeConfiguration(robot);
ndof = length(q0);
qs = zeros(count, ndof);


% Create the inverse kinematics solver. Because the xy Cartesian points are
% the only important factors of the end-effector pose for this workflow,
% specify non-zero weight for the fourth and fifth elements of the weight
% vector. all other elements are set to zero.

ik = robotics.InverseKinematics('RigidBodyTree', robot);
weights = [0, 0, 0, 1, 1, 0];
endEffector = 'tool';



qInitial = q0; % Use home configuration as the initial guess
for i = 1:count
    % Solve for the configuration satisfying the desired end effector
    % position
    point = points(i,:)
    qSol = ik(endEffector,trvec2tform(point),weights,qInitial);
    % Store the configuration
    qs(i,:) = qSol;
    % Start from prior solution
    qInitial = qSol;
end

qs=[p(1,:),p(2,:)]

figure
show(robot,qs(1,:)');
view(2)
ax = gca;
ax.Projection = 'orthographic';
hold on
plot(points(:,1),points(:,2),'k')
% axis([-0.1 0.7 -0.3 0.5])


framesPerSecond = 15;
r = robotics.Rate(framesPerSecond);
for i = 1:count
    show(robot,qs(i,:)','PreservePlot',false);
    drawnow
    waitfor(r);
end



