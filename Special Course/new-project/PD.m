clear all
close all
clc

syms k_Ti u_i n_i tau_i J_Mi f_Mi q_dd_Mi q_d_Mi real

n_i = 53;
J_Mi = 1340*10^(-7);
k_T = 0.24;
f_eff = 2.6*10^(-5);
g = 9.8;

act_mod = (k_Ti * u_i) - (1/n_i * tau_i) == J_Mi * q_dd_Mi + f_Mi*q_d_Mi;
q_dd_Mi= ((k_Ti * u_i) - (1/n_i * tau_i)  - f_Mi*q_d_Mi) / J_Mi;



% run('actuatorTest.m')
run('IndependentJoint.m')
clc
% Here we add the following:

syms K_P K_D s




G_s = (1/n^2)
N_s = Jeff_c*s^2 + f_eff*s + k_T*K_P + k_T*K_D*s
F_s = (K_D * s * k_T + K_P* k_T)

q1_test = (F_s * G_s)/ N_s(1)
% q2_test = (F_s * G_s)/ N_s(2)


syms omega_n zeta

% omega_n =  655.03890785716153861254900562852
% omega_n =  0.044232133946276766237923710560971
% omega_n = 0.011708062814524971573368792455476
omega_n = 15
zeta = 1
% qr_i = pi
qr_i = 0.35


K_P = []
K_D = []
K_I = []

% for i = 1:2
% 	K_P(i) = (omega_n^2 * Jeff_c(i))/k_T
% 	K_D(i) = ((2*omega_n*zeta*Jeff_c(i))/(k_T)-(f_eff)/(k_T))
% end


for i = 1:2
	K_P(i) = ((omega_n^2) * Jeff_c(i) + 2 * zeta * omega_n)/k_T
	K_D(i) = ( 1 + (2 * omega_n * zeta * Jeff_c(i)) - (f_eff) )/(k_T)
    K_I(i) = (omega_n^2)/k_T
end


Tlmax = max(Tl_c');
Tl = double(Tlmax(1))
Jeff = double(Jeff_c(1))
KP = double(K_P(1))
KD = double(K_D(1))
KI = double(K_I(1))

clear s

s = zpk('s');

% G_s = (1/n^2)
% N_s = double(Jeff_c(1))*s^2 + double(f_eff)*s + k_T*KP + k_T*KD*s
% F_s = (KD * s * k_T + KP* k_T)


G_s = (1/(n^2))
N_s = double(Jeff_c(1))*s^2 + double(f_eff)*s + k_T*KP + k_T*KD*s
F_s = (KD * s * k_T + KP* k_T + (KI/s))

% margin(q1_test)


num = [1]
den = [KD/KI KP/KI 1]
prefilt = tf(num,den)

% 
% q1_test = (F_s - G_s*Tl)/ N_s(1)
% 
% cltest = (1/n)*q1_test / (1+(1/n)*q1_test)
% 
% opt = stepDataOptions('StepAmplitude',0.35);
% step(cltest,opt)
% % figure()
% % margin(q1_test)



% opt = stepDataOptions('StepAmplitude',pi);
% step(q1_test,opt)
% q1_test = subs(q1_test, [K_D, K_P,s],[KD,KP,s])
res = sim('controller.slx')
% Tl2 = max(Tl_c(2))
% Tl_c



figure()
q1 = res.q1.signals.values
t = res.q1.time
plot(t,q1)
hold on
qr = res.qr.signals.values
t = res.qr.time
plot(t,qr)
grid on
title('Robot step-response.')
xlabel('Time [s]')
ylabel('Amplitude [rad]')


%%

% s = zkp('s')
H_s = ((KD * s) + KP + (KI/s)) * n * k_T
H2_s = (Tl * (1/n))
sys_ff = ((1/Jeff) * 1/s)
sys_cl = sys_ff/(1+sys_ff*f_eff)
total_sys_ff = (((H_s - H2_s)*sys_cl)*1/s)
total_sys_cl = total_sys_ff/(1 + total_sys_ff* (1/n))

figure()
margin(total_sys_cl)

total_sys_cl_n = (total_sys_ff*(1/n))/ (1 + total_sys_ff*(1/n))


% figure()
% opt = stepDataOptions('StepAmplitude',0.35);
% step(total_sys_cl,opt)
% step(total_sys_cl)
% hold on
% figure()
% opt = stepDataOptions('StepAmplitude',0.35);
% step(total_sys_cl_n,opt,10)
% grid on
% xaxis('rad')







%%

G_s = (1/(n^2))
N_s = double(Jeff_c(1))*s^2 + double(f_eff)*s + k_T*1 + k_T*1*s
F_s = (1 * s * k_T + 1* k_T + (1/s))

 
q1_test = (F_s - G_s*Tl)/ N_s(1)

margin(q1_test)
%%



G_s = (1/(n^2))
N_s = double(Jeff_c(1))*s^2 + double(f_eff)*s + k_T*KP + k_T*KD*s
F_s = (KD * s * k_T + KP* k_T + (KI/s))

q1_test = (F_s - G_s*Tl)/ N_s(1)



margin(q1_test)
