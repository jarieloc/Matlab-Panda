clear all
close all


syms d1 a1 d2 a2

robot = SerialLink( [ Revolute('a', a1) Revolute('a', a2) ],'name', 'my robot')
% robot.plot([0,0],'jvec','arrow','zoom',0.7)

syms q1 q2
A = robot.fkine([q1,q2])
tvec = A.transl'

x = tvec(1,1)
y = tvec(2,1)

xd = diff(x)
yd = diff(y)

o_0 = [0;0;0]
o_1 = [a1*cos(q1); a1*sin(q1); 0]
o_2 = [a1*cos(q1) + a2*cos(q1 + q2); a1*sin(q1) + a2*sin(q1 + q2); 0]


z0 = [0; 0; 1]
z1 = z0

J = [cross(z0, (o_2 - o_0)) cross(z1, (o_2 - o_1)); z0 z1]


%%

clear all
close all
q = linspace(0,pi*6,100)

f1 = cos(q)
f1d = diff(f1)
f1dd = diff(f1d)

plot(f1)
hold on
plot(f1d)
hold on
plot(f1dd)