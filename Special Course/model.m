clear all
close all
clc

%%

% Rigid body

body1 = robotics.RigidBody('body1');


% Create a joint and assign it to the rigid body

jnt1 = robotics.Joint('jnt1','revolute');

% Define the home position property of the joint.
jnt1.HomePosition = pi/4;

% Set the joint-to-parent transform using a homogeneuous transform
% 'tform'. Use the trvec2tform to convert translation vector to
% homogeneous transformation.
tform =trvec2tform([0.25, 0.25, 0]);
setFixedTransform(jnt1,tform)


%%
% Create a rigid body tree. This tree is initialized with a base coordinate
% frame attach bodies to.

robot = robotics.RigidBodyTree;
addBody(robot,body1,'base')

%%

% Create a second body. Define properties of this body and attach it to the
% first rigid body. Define the transformation relative to the previous body
% frame.

body2 = robotics.RigidBody('body2');
jnt2 = robotics.Joint('jnt2','revolute');
jnt2.HomePosition = pi/6;
tform2 = trvec2tform([1,0,0]);
setFixedTransform(jnt2,tform2);
body2.Joint = jnt2;
addBody(robot, body2, 'body1');

%%

body3 = robotics.RigidBody('body3');
body4 = robotics.RigidBody('body4');
jnt3 = robotics.Joint('jnt3','revolute');
jnt4 = robotics.Joint('jnt4','revolute');
tform3 = trvec2tform([0.6, -0.8, 0]) *eul2tform([-pi/2, 0, 0]); % User defined
tform4 = trvec2tform([1, 0, 0]); % User defined
setFixedTransform(jnt3,tform3);
setFixedTransform(jnt4,tform4);
jnt3.HomePosition = pi/4; % User defined
body3.Joint = jnt3
body4.Joint = jnt4
addBody(robot,body3,'body2'); % Add body3 to body2
addBody(robot,body4,'body2'); % Add body4 to body2

%%

% If you have a specific end effector that you care about for control,
% define it as a rigid body with a fixed joint. For this robot, add an end
% effector to body4 so that you can get transformation for it.

bodyEndEffector = robotics.RigidBody('endeffector');
tform5 = trvec2tform([0.5, 0, 0]); % User defined
setFixedTransform(bodyEndEffector.Joint,tform5);
addBody(robot,bodyEndEffector,'body4');

%%

config = robot.homeConfiguration
tform = getTransform(robot,config,'endeffector','body3')
%%
showdetails(robot)
view(2)
show(robot)



%%


x = -pi/2
y = 0
z = 0

rotz = [cos(x) -sin(x) 0; sin(x) cos(x) 0; 0 0 1]
roty = [cos(y) 0 sin(y); 0 1 0; -sin(y) 0 cos(y)]
rotx = [1 0 0; 0 cos(z) -sin(z); 0 sin(z) cos(z)]


transform = getTransform(robot,tform3,'body3')

tot = rotz*roty*rotx
eul2tform([-pi/2, 0, 0])



% Joints adjust their degrees from joint home configuration.