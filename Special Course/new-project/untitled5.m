clear
close all
clc
%%
clc

import ETS2.*
a1 = 1;
E = Rz('q1') * Tx(a1)

E.fkine(30,'deg')
E.teach

%%
clear all
clc
close all

import ETS2.*

robot = SerialLink( [ Revolute('d',0.15,'a', 0.8) Revolute('a', 0.8) ],'name', 'my robot')
robot.plot([0,0],'jvec','arrow','zoom',0.7)
title('Kinematic Model')
xlabel('X [m]')
ylabel('Y [m]')
zlabel('Z [m]')


p1 = linspace(0,pi,100)
p2 = linspace(0,deg2rad(150),100)
p3 = linspace(pi,0,100)
p4 = linspace(deg2rad(150),deg2rad(-150),100)

Tcollect = []
seg1 = []
seg2 = []
seg3 = []
seg4 = []
seg5 = []
%%
% robot.workspace
% Seg 1
for i= 1:length(p1)
    [T,A] = robot.fkine([p1(i), 0]);
    segrange = A(2).transl;
    seg1 = [segrange' seg1];
end
% Seg 2
for i= 1:length(p1)
    [T,A] = robot.fkine([p1(end),p2(i)]);
    segrange = A(2).transl;
    seg2 = [segrange' seg2];
end
% Seg 3
for i= 1:length(p1)
    [T,A] = robot.fkine([p3(i),p2(end)]);
    segrange = A(2).transl;
    seg3 = [segrange' seg3];
end
% Seg 4
for i= 1:length(p1)
    [T,A] = robot.fkine([p3(end),p4(i)]);
    segrange = A(2).transl;
    seg4 = [segrange' seg4];
end
% Seg 5
for i= 1:length(p1)
    [T,A] = robot.fkine([p1(i),p4(end)]);
    segrange = A(2).transl;
    seg5 = [segrange' seg5];
end

seg1 = seg1'
seg2 = seg2'
seg3 = seg3'
seg4 = seg4'
seg5 = seg5'

% res = [p1' p2' zeros(100,1)]
figure()
plot2(seg1,'ro')
hold on
plot2(seg2,'go')
hold on
plot2(seg3,'ro')
hold on
plot2(seg4,'gs')
hold on
plot2(seg5,'bs')
legend('seg1: q_1 rotation','seg2: q_2 rotation','seg3: q_1 rotation','seg4: q_2 rotation','seg5: q_1 rotation')
title('Reachable Workspace (Max/min)')
hold on
robot.plot([0,0])

figure()
plot2(seg1,'ro')
hold onr
plot2(seg2,'go')
hold on
plot2(seg3,'ro')
hold on
plot2(seg4,'gs')
hold on
plot2(seg5,'bs')
legend('seg1: q_1 rotation','seg2: q_2 rotation','seg3: q_1 rotation','seg4: q_2 rotation','seg5: q_1 rotation')
title('Reachable Workspace (Max/min)')
grid on
% hold on
% robot.plot([0,0])
%%
close all
robot2 = SerialLink( [ Revolute('d',0.15,'a', 0.8) Revolute('a', 0.8) ],'name', 'my robot')
syms q1 q2 real
syms ty tx real


TE = robot2.fkine([q1,q2])
e1 = tx == TE.t(1)
e2 = ty == TE.t(2)
[s1,s2] = solve( [e1 e2], [q1 q2] );
simplify(s1)
simplify(s2)


sol = robot2.ikine_sym(2)
length(sol)
s1 = sol{1};  % is one solution
q1 = s1(1)      % the expression for q1
q2 = s1(2)      % the expression for q2


s2 = sol{2};  % is one solution
q12 = s2(1)      % the expression for q1
q22 = s2(2)      % the expression for q2

%%


syms q1 q2 d1 a1 a2 d_1

alpha =[0 0]
d = [d_1 0]
theta = [q1 q2]
a = [a1 a2]

% A = [cos(theta(i)) -cos(alpha(i))*sin(theta(i)) sin(alpha(i))*sin(theta(i)) a(i)*cos(theta(i));
%         sin(theta(i)) cos(alpha(i))*cos(theta(i)) -sin(alpha(i))*cos(theta(i)) a(i)*sin(theta(i));
%         0 sin(alpha(i)) cos(alpha(i)) d(i);
%         0 0 0 1]
    
collect = []
T = eye(4)

for i = 1:2
    collect{i} = [cos(theta(i)) -cos(alpha(i))*sin(theta(i)) sin(alpha(i))*sin(theta(i)) a(i)*cos(theta(i));
        sin(theta(i)) cos(alpha(i))*cos(theta(i)) -sin(alpha(i))*cos(theta(i)) a(i)*sin(theta(i));
        0 sin(alpha(i)) cos(alpha(i)) d(i);
        0 0 0 1]
    T = T* collect{i}
end
A1 = collect{1}


J1 = [cross([0 0 1]',T(1:3,4)); [0 0 1]']
J2 = [cross(A1(1:3,3),T(1:3,4)-A1(1:3,4)) ; A1(1:3,3)]
J = simplify([J1 J2])
rank(J)

JS = []
js = 0

rankcollect = []
% a1 = 0.8; a2 = 0.8; d1 = 0.15;

for i=1:length(p1)
    JS = subs(J,[q1,q2,a1,a2,d1], [p1(i), 0, 0.8, 0.8, 0.15]);
    k = rank(JS);
    if k < 2
        js = js+1;
    end
end
rankcollect = [js rankcollect]

for i=1:length(p1)
    JS = subs(J,[q1,q2,a1,a2,d1], [p1(end), p2(i), 0.8, 0.8, 0.15]);
    k = rank(JS);
    if k < 2
        js = js+1;
    end
end
rankcollect = [js rankcollect]


for i=1:length(p1)
    JS = subs(J,[q1,q2,a1,a2,d1], [p3(i), p2(end), 0.8, 0.8, 0.15]);
    k = rank(JS);
    if k < 2
        js = js+1;
    end
end
rankcollect = [js rankcollect]


for i=1:length(p1)
    JS = subs(J,[q1,q2,a1,a2,d1], [p3(end), p4(i), 0.8, 0.8, 0.15]);
    k = rank(JS);
    if k < 2
        js = js+1;
    end
end
rankcollect = [js rankcollect]


for i=1:length(p1)
    JS = subs(J,[q1,q2,a1,a2,d1], [p1(i), p4(end), 0.8, 0.8, 0.15]);
    k = rank(JS);
    if k < 2
        js = js+1;
    end
end
rankcollect = [js rankcollect]




%%
[s,sd,sdd] = tpoly(0, 1, 50);
%%

% The patient's arm is now considered.


clear all
clc
close all

import ETS2.*

% figure(10)
robot = SerialLink( [ Revolute('d',0.15,'a', 0.8) Revolute('a', 0.8) Revolute('a', 0.45)],'name', 'my robot and forearm')
% robot.plot([0,0,0],'jvec','arrow','zoom',0.7)
% title('Kinematic Model /w Forearm')
% xlabel('X [m]')
% ylabel('Y [m]')
% zlabel('Z [m]')

M = robot.inertia([0,0,0])

%%
clear all
close all
clc

syms m1 r1 L1 m2 r2 L2 m3 r3 L3 m4 r4 L4

d1 = digits
d1 = digits(32)


I_1 = (1/12)*m1*(3*r1^2 + L1^2);
I_1yy = (1/2)*m1*r1^2;

I_2 = (1/12)*m2*(3*r2^2 + L2^2);
I_2yy = (1/2)*m2*r2^2;

I_3 = (1/12)*m3*(3*r3^2 + L3^2);
I_3yy = (1/2)*m3*r3^2;

I_4 = (1/12)*m4*(3*r4^2 + L4^2);
I_4yy = (1/2)*m4*r4^2;



I_1 = double(subs(I_1,[m1,r1,L1],[5, 0.1, 0.15]))
I_1yy = double(subs(I_1yy,[m1,r1],[5, 0.1]))

I_2 = double(subs(I_2,[m2,r2,L2],[5, 0.05, 0.7]))
I_2yy = double(subs(I_2yy,[m2,r2],[5, 0.04]))

I_3 = double(subs(I_3,[m3,r3,L3],[5, 0.05, 0.7]))
I_3yy = double(subs(I_3yy,[m3,r3],[5, 0.04]))

I_4 = double(subs(I_4,[m4,r4,L4],[4, 0.03, 0.4]))
I_4yy = double(subs(I_4yy,[m4,r4],[4, 0.03]))



