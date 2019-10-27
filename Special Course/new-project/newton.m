clear all
close all
clc

% syms d1 lc1 lc2 d_1 a1 a2

% lc1 = a1/2
% lc2 = a2/2

% linspace(0,4,10)
% % q1dd = [zeros(1,15)];
% q1dd = [linspace(0,4,5), ones(1,5)*4, linspace(4,0,5)];
% q2dd = [linspace(0,4,5), ones(1,5)*4, linspace(4,0,5)];
% 
% % q1dd = zeros(1,20)
% % q2dd = zeros(1,20)
% 
% q1d = cumtrapz(q1dd)
% q2d = cumtrapz(q2dd)
% 
% q1 = cumtrapz(q1d)
% q2 = cumtrapz(q1d)
% % 
% % length(q1dd)
% % length(q1d)
% % length(q1)


q1 = [linspace(0,pi/2,10), ones(1,5)*pi/2]
q2 = [linspace(0,pi/2,10), ones(1,5)*pi/2]

% q1d = linspace(0,1,20);
% q2d = linspace(0,1,20);

% q1d = zeros(1,20)
% q2d = zeros(1,20)

q1d = [0 diff(q1)];
q2d = [0 diff(q2)];

q1dd = [0 diff(q1d)];
q2dd = [0 diff(q2d)];

%%
d_1 = 0; a1 = 0.7; a2 = 0.7;


alpha =[0 0];
d = [d_1 0];
a = [a1 a2];

% [m2,r2,L2]

% [5, 0.05, 0.7]

m2 = 5;
r2 = 0.05
L2 = 0.7

% [m3,r3,L3]
m3 = 5;
r3 = 0.05
L3 = 0.7

% [m1,r1,L1],[5, 0.1, 0.15]

m1 = 5
r1 = 0.1
L1 = 0.15

% [m4,r4,L4],[4, 0.03, 0.4]
m4 = 4
r4 = 0.03
L4 = 0.4



% [5, 0.05, 0.7]

collect = [];
T = eye(4);
Tcol = []

Qout1 = [];
Qout2 = [];
qddr_out = [];
Qout = []

Mout1 = [];
Mout2 = [];

CGJFout1 = [];
CGJFout2 = [];

ta1= []
ta2= []
n = length(q1);

for j = 1:n
theta = [q1(j) q2(j)];

for i = 1:2
    collect{i} = [cos(theta(i)) -cos(alpha(i))*sin(theta(i)) sin(alpha(i))*sin(theta(i)) a(i)*cos(theta(i));
        sin(theta(i)) cos(alpha(i))*cos(theta(i)) -sin(alpha(i))*cos(theta(i)) a(i)*sin(theta(i));
        0 sin(alpha(i)) cos(alpha(i)) d(i);
        0 0 0 1];   
    T = T* collect{i};
    Tcol{i} = T;
end
A1 = double(Tcol{1});
A2 = double(Tcol{2});

R0 = double(A1(1:3,1:3))
R1 = double(A2(1:3,1:3))

R = {R0, R1};
alphaT = alpha;


% syms m1 r1 L1 m2 r2 L2 m3 r3 L3 m4 r4 L4
I_1 = (1/12)*m1*(3*r1^2 + L1^2);
I_1yy = (1/2)*m1*r1^2;

I_2 = (1/12)*m2*(3*r2^2 + L2^2);
I_2yy = (1/2)*m2*r2^2;

I_3 = (1/12)*m3*(3*r3^2 + L3^2);
I_3yy = (1/2)*m3*r3^2;

I_4 = (1/12)*m4*(3*r4^2 + L4^2);
I_4yy = (1/2)*m4*r4^2;


% Remeber the order of rigid bodies

I1 = [I_2 0 0; 0 I_2yy 0; 0 0 I_2];
I2 = [I_3 0 0; 0 I_3yy 0; 0 0 I_3];

% I{1}
% i=1
% 
lc1 = (0.7/2);
lc2 = (0.7/2);

l1 = 0.7;
l2 = 0.7;

m1 = 5;
m2 = 5;

% 
% r1 = 0.05;
% r2 = 0.05;
% 
% d_1 = 0.15


% syms qd1 qdd1 l1 lc1 l2 lc2 qd2 qdd2 g 
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
z0 = A1(1:3,3); % should just be [0;0;1], but A1 is the exact same.
z1 = A2(1:3,3);

R01 = R0;
R02 = R1;

% gload = [0; 0; g]


% r1c1 = [lc1*i; 0; 0]
% r2c1 = [(l1 - lc1)*i; 0; 0]
% r12 = [l1*i; 0; 0]
% 
% r2c2 = [lc2*i; 0; 0]
% r3c2 = [(l2 - lc2)*i; 0; 0]
% r23 = [l2*i 0 0]'

r1c1 = [lc1; 0; 0];
r2c1 = [(l1 -lc1); 0; 0];
r12 = [l1; 0; 0];

r2c2 = [lc2; 0; 0];
r3c2 = [(l2 -lc2); 0; 0];
r23 = [l2 0 0]';




z = [z0 z1]
R = {R01, R02};
I = {I1 I2};
m = [m1; m2];
re = {r12, r23};
rc = {-r1c1, -r2c1};

% syms qd1 qd2 qdd1 qdd2
% 
qd = [q1d(j); q2d(j)];
qdd = [q1dd(j); q2dd(j)];

[Q,t1,t2] = RNEA(z,R,I,m, alphaT, a, d,re,rc, theta, qd,qdd,2);
% qddr = inverseDynamics(Q,M,CGJF);
% CGF = RNEA(z,R,I,m,re,rc,qd,qdd0,2)
ta1 = [ta1 t1];
ta2 = [ta2 t2];

Qout = [Qout Q];
% Qout1 = [Qout1 Q(1,:)'];
% Qout2 = [Qout2 Q(2,:)'];


% Mout1 = [Mout1 M(1,:)'];
% Mout2 = [Mout2 M(2,:)'];
% 
% CGJFout1 = [CGJFout1 CGJF(1,:)'];
% CGJFout2 = [CGJFout2 CGJF(2,:)'];

% qddr_out = [qddr_out qddr];
end

%%
close all

robot2 = SerialLink( [ Revolute('a', 0.7,'m',5,'r',[-0.35; 0; 0], 'I', I1) Revolute('a', 0.7,'m',5,'r',[-0.35; 0; 0], 'I', I2) ],'name', 'my robot')
robot.gravity = [0;0;-9.8];

torque = robot2.rne([q1',q2'],[q1d',q2d'],[q1dd',q2dd']);

% Forward Kinematics 
% [TE, A] = robot2.fkine([q1',q2'])
% seg1 = []
% for i= 1:length(q1)
%     [T,A] = robot2.fkine([q1(i), q2(i)]);
%     segrange = A(1).transl;
%     seg1 = [segrange' seg1];
% end

% Torque algorithm vs torque toolbox
tres1 = double(Qout(1:3,:));
tres2 = double(Qout(4:6,:));
tres1 = tres1';
tres2 = tres2';


figure()
subplot(2,2,1),plot(tres1(:,3)); hold on; plot(tres2(:,3)); hold off; grid on;
% subplot(2,2,1),plot(tres1(:,1),tres1(:,2)); hold on; plot(tres2(:,1),tres2(:,2)); hold off; grid on;
% subplot(2,2,1),plot(Qout(:,:)'); hold off; grid on;
legend('\tau_1 computed','\tau_2 computed')
xlabel('samples')
ylabel('[N?m]')
title('Computed RNEA')

subplot(2,2,2),plot(1:length(torque),torque(:,1),'r', 1:length(torque), torque(:,2),'b'); grid on; 
legend('\tau_1 ToolB','\tau_2 ToolB')
xlabel('samples')
ylabel('[N?m]')
title('Robotics ToolBox RNEA')

subplot(2,2,3),plot(q1); hold on; plot(q1d); hold on; 
plot(q1dd); grid on; title('Joint 1');
legend('q1','$\dot{q1}$','$\ddot{q1}$','Interpreter','latex');
xlabel('samples')
ylabel('[rad]')
hold off

subplot(2,2,4),plot(q2); legend('q1'); hold on; plot(q2d); legend('\dot{q1}'); hold on; 
plot(q2dd); legend('q2','$\dot{q2}$','$\ddot{q2}$','Interpreter','latex'); grid on;
title('Joint 2')
xlabel('samples')
ylabel('[rad]')
hold off


%%







figure()
subplot(2,2,1),plot(tres1','r');
grid on
hold on
hold off
title('\tau 1')


subplot(2,2,2),plot(tres2','b');
grid on
hold on
hold off
title('\tau 2')

subplot(2,2,3),plot(Qout','b');
grid on
% hold on
% plot(Qout1(2,:));
% hold off
title('Q matrix')

subplot(2,2,4),plot(torque);
grid on
title('toolbox tau')





%%
subplot(2,2,2),plot(Qout2(1,:),'r');
grid on
hold on
plot(Qout2(2,:));
hold off
title('\tau 2')

subplot(2,2,3),plot(q1)
hold on
plot(q1d)
hold on
plot(q1dd)
grid on
title('q1')
hold off

subplot(2,2,4),plot(q2)
hold on
plot(q2d)
hold on
plot(q2dd)
grid on
title('q2')
hold off

% grid on



figure()
subplot(2,2,1),plot(Mout1(1,:),'r');
grid on
hold on
plot(Mout1(2,:));
hold off
title('M \tau 1')

subplot(2,2,2),plot(Mout2(1,:),'r');
grid on
hold on
plot(Mout2(2,:));
hold off
title('M \tau 2')

subplot(2,2,3),plot(CGJFout1(1,:),'r');
grid on
hold on
plot(CGJFout1(2,:));
hold off
title('CGJF \tau 1')

subplot(2,2,4),plot(CGJFout2(1,:),'r');
grid on
hold on
plot(CGJFout2(2,:));
hold off
title('CGJF \tau 2')



figure()
plot(ta1(1,:),ta1(2,:))
grid on
hold on
% plot(ta1(2,:))
% hold on
% plot(ta1(3,:))

figure()
plot(ta2(1,:),ta2(2,:))
grid on
% hold on
% plot(ta2(2,:))

% figure()
% plot(qddr_out(1,1:end))
% grid on
% hold on
% 
% plot(qddr_out(2,1:end))
% grid on
% 
% hold off



% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 



% qdd1 = zeros(1,7)
% qdd2 = zeros(1,7)











% run('rne')
% CG = [tau1; tau2]

% make into a function



% Forward
% 
% w(i) = (inv(R(i-1,i))') * w(i-1) + b(i) * qd(i)
% 
% alpha(i) = (R(i,i-1)') * alpha(i-1) + b(i)*qdd(i) + cross(w(i),(b(i)*q(d)))
% 
% ae(i) = (inv(R(i-1,i))') * ae(i-1) + cross(wd(i),r(i,i+1)) + cross(w(i),r(i,i+1))
% 
% ac(i) = (inv(R(i-1,i))') * ac(i-1) + cross(wd(i),r(i,ci)) + cross(w(i),r(i,ci))
% 
% 
% 
% 


% % Backward
% 
% f(i) = R(i,i+1)*f(i+1) + m(i)*ac(i) - m(i)*g(i)
% 
% tau(i) = R(i,i+1) - cross(f(i),r(i,ci)) +  cross((R(i,i+1)*f(i+1)) ,r(i+1,ci)) + alpha(i) + cross( w(i), (I(i)*w(i)) )
% 
