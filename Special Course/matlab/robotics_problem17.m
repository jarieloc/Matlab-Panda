clear all
close all
clc
global GM m1 m2 m3 m4 L1 L2 dd2 L3 a4 I2 I3 I4 I1yy I2zz I3yy g n kT feff d1;
d1 = 1.6
a4 = 1.1

% From problem 4
T1 = [0 0 0 0;0 0 0 -0.5; -1 0 0 0; 0 0 0 1]; 
T2 = [0 0 0 0.35;0 0 0 0.5; -1 0 0 0; 0 0 0 1]; 
T3 = [0 0 0 0.7;0 0 0 -0.5; -1 0 0 0; 0 0 0 1]; 
T4 = [0 0 0 1.05 ;0 0 0 0.5; -1 0 0 0; 0 0 0 1]; 
T5 = [0 0 0 1.4 ;0 0 0 -0.5; -1 0 0 0; 0 0 0 1]; 
Ttask = [1 0 0 1.5;0 1 0 0; 0 0 1 0.2; 0 0 0 1]; 
a_4 = 1.10; d_1 = 1.6; 
TN = {T1 T2 T3 T4 T5}; collect = []; tp_cell = [];
for i = 1:5
    collect = Ttask*TN{1,i};
    tp_cell{i} = collect;
end
q1 = []; q2 = []; q3 = []; q4 = []; tp = [];
for i = 1:5
    tp = tp_cell{i};
    q1(i) =  atan2( tp(2,4), tp(1,4));
    q2(i) =  atan2( sin(q1(i))*(tp(2,4)-a_4*tp(2,1))+cos(q1(i))*(tp(1,4)-a_4*tp(1,1)),tp(3,4)-d_1-a_4*tp(3,1));
    q3(i) = (sin(q1(i))*(tp(2,4)-a_4*tp(2,1))+cos(q1(i))*(tp(1,4)-a_4*tp(1,1)))/sin(q2(i));
    q4(i) = atan2( tp(3,1), ((-sin(q1(i))* tp(2,4) - cos(q1(i)) * tp(1,4) + q3(i) * sin(q2(i)))/a_4) ) - q2(i)+2*pi;
end
T = [q1;q2;q3;q4];
% Duration is 2 seconds
t0 = 0; tf = 2; syms q0; v0 = 0; a0 = 0; syms  qf; vf = 0; af = 0; 
a_mat_1 = inv([1 t0 t0^2 t0^3 t0^4 t0^5; 
               0 1 2*t0 3*t0^2 4*t0^3 5*t0^4; 
               0 0 2 6*t0 12*t0^2 20*t0^3; 
               1 tf tf^2 tf^3 tf^4 tf^5;
               0 1 2*tf 3*tf^2 4*tf^3 5*tf^4; 
               0 0 2 6*tf 12*tf^2 20*tf^3]); 
traj_vec = [q0; v0; a0; qf; vf; af]; 
segments = []; segments_collect = []; 
segments_collect2 = []; 
for i = 1:4
    q1seg = double(subs(a_mat_1 * traj_vec, [q0,qf], [q1(i), q1(i+1)]));
    q2seg = double(subs(a_mat_1 * traj_vec, [q0,qf], [q2(i), q2(i+1)]));
    q3seg = double(subs(a_mat_1 * traj_vec, [q0,qf], [q3(i), q3(i+1)]));
    q4seg = double(subs(a_mat_1 * traj_vec, [q0,qf], [q4(i), q4(i+1)]));
    segments = [q1seg'; q2seg'; q3seg'; q4seg']; 
    segments_collect{i} = segments; 
    segments_collect2 = [segments_collect2 segments];
end
seg1 = [];seg2 = [];seg3 = [];seg4 = [];
seg1 = segments_collect{1}; seg2 = segments_collect{2}; seg3 = segments_collect{3}; seg4 = segments_collect{4};
GM = [seg1;seg2;seg3;seg4];

% Problem 17
GM = fliplr(GM)
sim('problem17.slx')

vis4link(M,1,18)

t = qv.time;
qv1 = qv.signals.values;
pt = p.time;
pv = p.signals.values;
figure()
plot(t,qv1(:,1), t,qv1(:,2), t,qv1(:,3), t,qv1(:,4),'LineWidth',3);
grid on;
% legend()
% axis([0 8 -1 4])
title('A7','Interpreter','latex')
xlab = xlabel('Time $[sec]$','Interpreter','latex')
ylab = ylabel('$(q_1, q_2, q_4)[rad]$ and $(q_3)[m]$','Interpreter','latex')
set(xlab,'FontSize',14)
set(ylab,'FontSize',14)

hold on
plot([0 2 4 6 8], q1,'r--x',[0 2 4 6 8], q2,'b--x',[0 2 4 6 8], q3,'k--x',[0 2 4 6 8], q4,'y--x','LineWidth',0.25)
lgd = legend('$q_1$(sim)','$q_2$(sim)','$q_3$(sim)','$q_4$(sim)','$q_1$(est)','$q_2$(est)','$q_3$(est)','$q_4$(est)','Location', 'bestoutside','Interpreter','latex')
set(lgd,'FontSize',14)
% lgd2 = legend('$q_1$(est)','$q_2$(est)','$q_3$(est)','$q_4$(est)','Location', 'bestoutside','Interpreter','latex')


% xlab = xlabel('Time [sec]')
% ylab = ylabel('(q_1, q_2, q_4)[rad] and (q_3)[m]')
% lgd.NumColumns = 4;
figure()
plot(pv(:,1),pv(:,2),'LineWidth',1);
title('A8','Interpreter','latex')
xlab = xlabel('$P_x$','Interpreter','latex')
ylab = ylabel('$P_y$','Interpreter','latex')
set(xlab,'FontSize',14)
set(ylab,'FontSize',14)
grid on
lgd2 = legend('$(P_x, P_y)$','Interpreter','latex')
set(lgd2,'FontSize',14)

