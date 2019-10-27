clear all
close all
clc 

run('IndependentJoint')
n_i = 53;
k_T = 0.24;
f_eff = 2.6*10^(-5);
g = 9.8;
D_q = vpa(subs(Dq,[I2_temp,I3_temp,m1,m2,lc1,lc2,l1,l2],[I2res,I3res,5,5,(0.7/2),(0.7/2),0.7,0.7]),4);
%%

M = D_q +  diag(Jeff_c)
B = f_eff
C_q = vpa(subs(C,[I2_temp,I3_temp,m1,m2,lc1,lc2,l1,l2],[I2res,I3res,5,5,(0.7/2),(0.7/2),0.7,0.7]),4)
Gload = g_load

%%
% 
% s = zpk('s')
% 
% 
% 

syms fx fy fz nx ny nz
Fvec = [fx fy fz nx ny nz]'

tau_res = J' * Fvec


Jt = J'

KI = 1
KP = 1

%%
s = zpk('s')
q_d = [q1_d q2_d]'

MI = inv(M)
sys1 = vpa( simplify((MI * (-C_q*q_d)) - (MI*Gload)) ,4)
sys2 = vpa(simplify(MI),4)

% MI*C*

% qd = [q1d; q2d]'
% qdd = [q1dd q2dd]'
% aq = qdd
% 
% u = M*aq + C*qd + Gload
% aq_res = MI*(u - C*qd - Gload)

%%

% aq = -K0*q - K1*qd + r

% aq + K0*q + K1*qd = r

% K0 = [omega1 omega2]'
% K1 = [2*omega1 2*omega2]'

%%
i