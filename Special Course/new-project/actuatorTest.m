clear all
close all
clc


% (k_Ti * u_i)        % torque constant * control voltage

% (1/n_i * tau_i)     % 1 / gear ratio * load torque

% J_Mi * q_dd_Mi      % mass moment of inertia of the motor

% f_Mi*q_d_Mi         % motor angle [radians]

syms k_Ti u_i n_i tau_i J_Mi f_Mi q_dd_Mi q_d_Mi real

n_i = 53;
J_Mi = 1340*10^(-7);
k_T = 0.24;
f_eff = 2.6*10^(-5);
g = 9.8;

act_mod = (k_Ti * u_i) - (1/n_i * tau_i) == J_Mi * q_dd_Mi + f_Mi*q_d_Mi;
q_dd_Mi= ((k_Ti * u_i) - (1/n_i * tau_i)  - f_Mi*q_d_Mi) / J_Mi;





syms q1 q2 d1 lc1 lc2 d_1 a1 a2

% lc1 = a1/2
% lc2 = a2/2

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
syms l1 l2
A1c = subs(A1, [a1,a2],[l1,l2]);


J1 = [cross([0 0 1]',T(1:3,4)); [0 0 1]']
J2 = [cross(A1(1:3,3),T(1:3,4)-A1(1:3,4)) ; A1(1:3,3)]


% Setup
J1_v = [cross([0 0 1]',A1(1:3,4))];
J1_w = [0 0 1]'
J2_v = [cross(A1c(1:3,3),T(1:3,4)-A1c(1:3,4))]
J2_w = [A1c(1:3,3)]


% Collect
J1v = [J1_v zeros(3,1)]
J1w = [J1_w zeros(3,1)]

% Collect
J1_v = [cross([0 0 1]',A1c(1:3,4))]
J2v = [(J1_v+J2_v) J2_v]
J2w = [(J2_v+J2_v) J2_v]

% Substitute
J1c_v = subs(J1v,[a1,a2],[lc1,lc2])
J1c_w = subs(J1w,[a1,a2],[lc1,lc2])
J2c_v = subs(J2v , [a1,a2],[lc1,lc2])
J2c_w = subs(J2w , [a1,a2],[lc1,lc2])


J = simplify([J1 J2]);
rank(J);

%%

syms m1 r1 L1 m2 r2 L2 m3 r3 L3 m4 r4 L4


I_1 = (1/12)*m1*(3*r1^2 + L1^2)
I_1yy = (1/2)*m1*r1^2

I_2 = (1/12)*m2*(3*r2^2 + L2^2)
I_2yy = (1/2)*m2*r2^2

I_3 = (1/12)*m3*(3*r3^2 + L3^2)
I_3yy = (1/2)*m3*r3^2

I_4 = (1/12)*m4*(3*r4^2 + L4^2)
I_4yy = (1/2)*m4*r4^2



% I_1 = double(subs(I_1,[m1,r1,L1],[5, 0.1, 0.15]))
% I_1yy = double(subs(I_1yy,[m1,r1],[5, 0.1]))
% 
% I_2 = double(subs(I_2,[m2,r2,L2],[5, 0.05, 0.7]))
% I_2yy = double(subs(I_2yy,[m2,r2],[5, 0.04]))
% 
% I_3 = double(subs(I_3,[m3,r3,L3],[5, 0.05, 0.7]))
% I_3yy = double(subs(I_3yy,[m3,r3],[5, 0.04]))
% 
% I_4 = double(subs(I_4,[m4,r4,L4],[4, 0.03, 0.4]))
% I_4yy = double(subs(I_4yy,[m4,r4],[4, 0.03]))
% 
% 
% 
I1 = [I_1 0 0; 0 I_1yy 0; 0 0 I_1]
I2 = [I_2 0 0; 0 I_2yy 0; 0 0 I_2]
I3 = [I_3 0 0; 0 I_3yy 0; 0 0 I_3]
I4 = [I_4 0 0; 0 I_4yy 0; 0 0 I_4]
% 
% I1m = max(max(I1))
% I2m = max(max(I2))
% I3m = max(max(I3))
% I4m = max(max(I4))
% 
% I_imax = [I1m; I2m; I3m; I4m]
% D_hat = diag(I_imax)

syms q_d


% size(I2)
% p1 = (I2 * [eye(3) zeros(3); zeros(3) zeros(3)]

% p1 = [I2 zeros(3); zeros(3) zeros(3)]
% p2 = [I3 I3; I3 I3]

% I_col = p1 + p2
syms I2_temp I3_temp
I_col = [I2_temp 0; 0 0] + [I3_temp I3_temp; I3_temp I3_temp]

D = (m1*J1c_v' * J1c_v + m2*J2c_v' * J2c_v) + I_col;
Dq = simplify(D)


d11 = Dq(1,1)
d12 = Dq(1,2)
d22 = Dq(2,2)
d21 = d12

%%

% I2_temp = I2
% I3_temp = I3


c111 = (1/2) * diff(d11,q1)
c121 = (1/2) * diff(d11,q2)
c211 = c121
c221 = diff(d12,q2) - ((1/2) * diff(d22,q1))
c112 = diff(d21,q1) - ((1/2) * diff(d11,q2))
c122 = (1/2) * diff(d22,q1)
c212 = c122
c222 = (1/2) * diff(d22,q2)


% subs(c111,[I2_temp,I3_temp],[I2,I3])
% subs(c121,[I2_temp,I3_temp],[I2,I3])
% subs(c211,[I2_temp,I3_temp],[I2,I3])
% subs(c221,[I2_temp,I3_temp],[I2,I3])
% subs(c112,[I2_temp,I3_temp],[I2,I3])
% subs(c122,[I2_temp,I3_temp],[I2,I3])
% subs(c212,[I2_temp,I3_temp],[I2,I3])
% subs(c222,[I2_temp,I3_temp],[I2,I3])

%%
% P1 = m1 * g * 
% 
% rc1 = [cross([0 0 1]',A1(1:3,4))]
% rc2 = [cross(A1c(1:3,3),T(1:3,4)-A1c(1:3,4))]
% rc12 = rc1 + rc2

syms g

P1 = m1*g*d_1
P2 = m2*g*d_1

P = P1 + P2

phi_1 = diff(P,q1)
phi_2 = diff(P,q2)

g_load = [phi_1;phi_2]
C = simplify([(c111 + c211) (c121 + c221);  (c112+c212) (c122+c222)])

syms q1_d q2_d q1_dd q2_dd
q_dd = [q1_dd;q2_dd]
q_d = [q1_d; q2_d]

torque = D*[q1_dd;q2_dd] + C*q_d + g*q_dd
torque = simplify(torque)


%%

% Independent joint control

q1t = linspace(0,0.35,100);
q2t = linspace(0,0.35,100);

q1d = [0 diff(q1t)]
q2d = [0 diff(q2t)]

q1dd = [0 0 diff(q1d)]
q2dd = [0 0 diff(q2d)]

% q1d = zeros(1,length(q1t));
% q2d = zeros(1,length(q1t));
% % 
% q1dd = zeros(1,length(q1t));
% q2dd = zeros(1,length(q1t));



% p2 = linspace(0,deg2rad(150),100)
% p3 = linspace(pi,0,100)
% p4 = linspace(deg2rad(150),deg2rad(-150),100)

J_M = 1340 * 10^(-7)

I_1 = double(subs(I_1,[m1,r1,L1],[5, 0.1, 0.15]))
I_1yy = double(subs(I_1yy,[m1,r1],[5, 0.1]))

I_2 = double(subs(I_2,[m2,r2,L2],[5, 0.05, 0.7]))
I_2yy = double(subs(I_2yy,[m2,r2],[5, 0.04]))

I_3 = double(subs(I_3,[m3,r3,L3],[5, 0.05, 0.7]))
I_3yy = double(subs(I_3yy,[m3,r3],[5, 0.04]))

I_4 = double(subs(I_4,[m4,r4,L4],[4, 0.03, 0.4]))
I_4yy = double(subs(I_4yy,[m4,r4],[4, 0.03]))



I2res = I_2 - I_2yy
I3res = I_3 - I_3yy

% collections = []
d11 = []
d22 = []

% m1 and m2 are substituted with m2 and m3
% lc1 and lc2 are half of the link length

Tl_c = []
for i=1:length(q1t)
    Dtest = vpa(subs(Dq,[q1,q2,I2_temp,I3_temp,m1,m2,lc1,lc2,l1,l2],[q1t(i),q2t(i),I2res,I3res,5,5,(0.7/2),(0.7/2),0.7,0.7]),4);
    Tl_col = vpa(subs(torque,[q1,q2,I2_temp,I3_temp,m1,m2,lc1,lc2,l1,l2,q1_d, q2_d,q1_dd, q2_dd, g],[q1t(i),q2t(i),I2res,I3res,5,5,(0.7/2),(0.7/2),0.7,0.7, q1d(i), q2d(i), q1dd(i), q2dd(i), 9.8]),4);
%     simplify(Dtest);
    d11 = [Dtest(1,1) d11];
    d22 = [Dtest(1,1) d22];
    
    Tl_c = [Tl_col Tl_c];
end
% plot(collections)


d11_max = vpa(simplify(max(d11)),4)
d22_max = vpa(simplify(max(d22)),4)
n = 53
Jeff_c = vpa((1/n^2)*[d11_max d22_max]+J_M*[1 1],4)






