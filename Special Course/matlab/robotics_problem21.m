clear all
close all
clc

% GM matrix from problem 18
T1 = [0 0 0 0;0 0 0 -0.5; -1 0 0 0; 0 0 0 1]; 
T2 = [0 0 0 0.35;0 0 0 0.5; -1 0 0 0; 0 0 0 1]; 
T3 = [0 0 0 0.7;0 0 0 -0.5; -1 0 0 0; 0 0 0 1]; 
T4 = [0 0 0 1.05 ;0 0 0 0.5; -1 0 0 0; 0 0 0 1]; 
T5 = [0 0 0 1.4 ;0 0 0 -0.5; -1 0 0 0; 0 0 0 1]; 
Ttask = [1 0 0 1.5;0 1 0 0; 0 0 1 0.2; 0 0 0 1]; 
a_4 = 1.10; 
d_1 = 1.6; 
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
t0 = 0; tf = 2; syms q0; v0 = 0; a0 = 0; syms  qf;  vf = 0;  af = 0; 
a_mat_1 = inv([1 t0 t0^2 t0^3 t0^4 t0^5; 
               0 1 2*t0 3*t0^2 4*t0^3 5*t0^4; 
               0 0 2 6*t0 12*t0^2 20*t0^3; 
               1 tf tf^2 tf^3 tf^4 tf^5;
               0 1 2*tf 3*tf^2 4*tf^3 5*tf^4; 
               0 0 2 6*tf 12*tf^2 20*tf^3]); 
traj_vec = [q0; v0; a0; qf; vf; af]; 
segments = []; segments_collect = []; 
for i = 1:4
    q1seg = double(subs(a_mat_1 * traj_vec, [q0,qf], [q1(i), q1(i+1)]));
    q2seg = double(subs(a_mat_1 * traj_vec, [q0,qf], [q2(i), q2(i+1)]));
    q3seg = double(subs(a_mat_1 * traj_vec, [q0,qf], [q3(i), q3(i+1)]));
    q4seg = double(subs(a_mat_1 * traj_vec, [q0,qf], [q4(i), q4(i+1)]));
    segments = [q1seg'; q2seg'; q3seg'; q4seg']; 
    segments_collect{i} = segments; 
end
seg1 = [];seg2 = [];seg3 = [];seg4 = [];
seg1 = segments_collect{1};
seg2 = segments_collect{2};
seg3 = segments_collect{3};
seg4 = segments_collect{4};

syms p0 pf
cart_vec = [p0; v0; a0; pf; vf; af]; pseg = []; pseg_collect = [];
for i = 1:4
    tp = tp_cell{i};
    tp2 = tp_cell{i+1};
    pseg = []
    px = double(subs(a_mat_1 * cart_vec, [p0,pf], [tp(1,4), tp2(1,4)]))
    py = double(subs(a_mat_1 * cart_vec, [p0,pf], [tp(2,4), tp2(2,4)]))
    pseg = [px';py']
    pseg_collect{i} = pseg
end

pseg1 = pseg_collect{1}; pseg2 = pseg_collect{2}; pseg3 = pseg_collect{3}; pseg4 = pseg_collect{4};
GM = [pseg1;pseg2;pseg3;pseg4];
GM = fliplr(GM)


global GM m1 m2 m3 m4 L1 L2 dd2 L3 a4 I2 I3 I4 I1yy I2zz I3yy g n kT feff d1;
d1 = 1.6
a4 = 1.1

L1 = 0.7;
r1 = 0.04;
L2 = 1.68;
b2 = 0.2;
L3 = 1.7;
b3 = 0.15;
a4 = 1.1;
m1 = 5;
m2 = 8;
m3 = 5;
m4 = 2;
I1yy = m1*((r1^2)/2)
I2 = (m2/12)*((b2^2)+(L2^2))
I2zz = (m2/12)*((b2^2)+(b2^2))
I3 = (m3/12)*((b3^2)+(L3^2))
I3yy = (m3/12)*((b3^2)+(b3^2))
I4 = m4*(a4^2)/12
dd2 = 0.36;


% Variables taken from project report

g = 9.8; kT = 0.24; feff = 2.6*10^(-5);
n = 53; omega = 15; zeta = 1;

% jeff_c from casper's problem12.mlx
Jeff1 = 0.0192; Jeff2 = 0.0192; Jeff3 = 0.00263; Jeff4 = 4.21*10^(-4);

% KI1 = omega^2/kT;
% KI2 = omega^2/kT;
% KI3 = omega^2/kT;
% KI4 = omega^2/kT;

KI1 = 0
KI2 = 0
KI3 = 0
KI4 = 0

Kp1 = (2 * zeta * omega + Jeff1 * omega^2)/kT;
Kp2 = (2 * zeta * omega + Jeff2 * omega^2)/kT;
Kp3 = (2 * zeta * omega + Jeff3 * omega^2)/kT;
Kp4 = (2 * zeta * omega + Jeff4 * omega^2)/kT;

KD1 = (1 + 2 * zeta * omega * Jeff1 - feff)/kT;
KD2 = (1 + 2 * zeta * omega * Jeff2 - feff)/kT;
KD3 = (1 + 2 * zeta * omega * Jeff3 - feff)/kT;
KD4 = (1 + 2 * zeta * omega * Jeff4 - feff)/kT;


sim('trc_dyn.mdl')
% qr
% 
% % test.signals.values

% vis4link(M,1,18)



figure()
plot3(p(:,1),p(:,2),p(:,3),'LineWidth',1.5);
title('Problem 21 Cartesian Space (With Initial Joint Conditions)','Interpreter','latex')
xlab = xlabel('$P_x$','Interpreter','latex')
ylab = ylabel('$P_y$','Interpreter','latex')
zlab = zlabel('$P_z$','Interpreter','latex')
set(xlab,'FontSize',14)
set(ylab,'FontSize',14)
set(zlab,'FontSize',14)
grid on

figure()
plot(p(:,1),p(:,2),'LineWidth',2);
title('A11','Interpreter','latex')
xlab = xlabel('$P_x$','Interpreter','latex')
ylab = ylabel('$P_y$','Interpreter','latex')
set(xlab,'FontSize',14)
set(ylab,'FontSize',14)
grid on
hold on
for i = 1:4
    tp = tp_cell{i};
    tp2 = tp_cell{i+1};
    pseg = []
    px = [tp(1,4) tp2(1,4)]
    py = [tp(2,4) tp2(2,4)]
    plot(px,py,'--','LineWidth',2)
    hold on
end
lgd2 = legend('$(P_x, P_y)$','seg1','seg2','seg3','seg4','Interpreter','latex')
set(lgd2,'FontSize',14);