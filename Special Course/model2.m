clear all
close all
clc

%%
robot = robotics.RigidBodyTree('DataFormat','column','MaxNumBodies',3);


% Specfy arm lengths for the robt arm
L1 = 0.3;
L2 = 0.3;
%%
% Add 'link1' body with 'joint1' joint.

body = robotics.RigidBody('link1');
joint = robotics.Joint('joint1', 'revolute');
setFixedTransform(joint,trvec2tform([0 0 0]));
joint.JointAxis = [0 0 1];
body.Joint = joint;
addBody(robot, body, 'base');

%%
%Add 'link2' body with 'joint2' joint.

body = robotics.RigidBody('link2');
joint = robotics.Joint('joint2','revolute');
setFixedTransform(joint, trvec2tform([L1,0,0]));
joint.JointAxis = [0 0 1];
body.Joint = joint;
addBody(robot, body, 'link1');

%%
% Add 'tool' end effector with 'fix1' fixed joint.

body = robotics.RigidBody('tool');
joint = robotics.Joint('fix1','fixed');
setFixedTransform(joint, trvec2tform([L2, 0, 0]));
body.Joint = joint;
addBody(robot, body, 'link2');


%%
showdetails(robot)
show(robot)



%%

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
    qs(i,:) = qSol
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
axis([-0.1 0.7 -0.3 0.5])


framesPerSecond = 15;
r = robotics.Rate(framesPerSecond);
for i = 1:count
    show(robot,qs(i,:)','PreservePlot',false);
    drawnow
    waitfor(r);
end
