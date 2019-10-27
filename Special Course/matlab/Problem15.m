clear all
close all
g=9.82;
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
delta2 = 0.36;
% Taget direkte fra Annex B.
%Foelgende skal bruges til problem 9, for at bestemme de effektive moments of inertia
I1 = (m1/12)*(3*(r1^2)+(L1^2))
size(I1)
I1yy = m1*((r1^2)/2)
I2 = (m2/12)*((b2^2)+(L2^2))
I2zz = (m2/12)*((b2^2)+(b2^2))
I3 = (m3/12)*((b3^2)+(L3^2))
I3yy = (m3/12)*((b3^2)+(b3^2))
I4 = m4*(a4^2)/12

syms q1 q2 q3 q4

K0 = m2 * ((0.5 * L2-delta2)^2) + m3 * ((0.5*L3))^2
K1 = 0.5 * (I4 + (1/4) * m4 * a4^2)
K2 = 0.5 * (I2 - I2zz + I3 - I3yy + K0)
K3 = I2 + I3 + K0 + (2 * K1)
K4 = m3 + m4
K5 = 0.5 * ((2 * I1yy) + I2zz + I3yy + K3)

f1 = vpa((1/2) * K4 * q3^2 - (1/2)  * m3 * L3 * q3,3)
f2 = vpa(m4 * a4 * q3,3)
f = vpa((1/2) * a4 * m4 * cos(q4),3)

D11 = vpa(K5 + f1 - ((K2 + f1) * cos(2 * q2)) + K1 * cos(2 * (q2 + q4)) - f2 * cos(q2 + q4)*sin(q2),2)
D = vpa([D11 0 0 0;
     0 K3+2*f1+f2*sin(q4) f 2*K1+0.5*f2*sin(q4);
     0 f K4 f;
     0 2*K1+0.5*f2*sin(q4) f 2*K1],3)
 d_11_max=vpa(subs(D(1,1),[q2,q3,q4],[pi/2,3,pi/2]),3)
 d_22_max=vpa(subs(D(2,2),[q3,q4],[3,pi/2]),3)
 d_33_max=D(3,3)
 d_44_max=D(4,4)

h = vpa(g*[0;
       0.5*m4*a4*cos(q2+q4)+(m2*(delta2-0.5*L2)+m3*(0.5*L3-q3)-m4*q3)*sin(q2);
       (m3+m4)*cos(q2);
       0.5*a4*m4*cos(q2+q4)],3)
T_L2=vpa(subs(h(2),[q2,q3,q4],[pi/2,3,pi/2]),3)
T_L3=vpa(subs(h(3),[q2],[pi/6]),3)
T_L4=vpa(subs(h(4),[q2,q4],[pi/2,pi/2]),3)

n= 53;
Jm = 1340*10^-7;
jeff_c = vpa((1/n^2)*[d_11_max d_22_max d_33_max d_44_max]+Jm*[1 1 1 1])
kT=0.24;
f_eff = 2.6*10^-5;
wn = 15;
damp = 1;
Kd_c = (1+jeff_c*2*damp*wn-f_eff)/kT
Kp_c = (jeff_c*wn^2+2*damp*wn)/kT
Ki_c = wn^2/kT
q_c = [0.35;0.35;0.35;0.35]
Simulationtime = '0.75';
% Kp=2;
% Kd=3;
% jeff =2;
% q=1;

% Figur A1. Plot med daempning paa 1
counter_i= 1
TL = 0;
Kp=double(Kp_c(counter_i))
Kd=double(Kd_c(counter_i))
Ki=double(Ki_c)
jeff= double(jeff_c(counter_i))
q=q_c(counter_i)
set_param('Motor_load_and_controller_PID','StopTime', [Simulationtime],'SimulationCommand','update')
sim('Motor_load_and_controller_PID')
output_q1=simout;
output_q1.data(20)
plot(output_q1.time, output_q1.data,'o')
hold on

counter_i= 2
TL=double(T_L2);
Kp=double(Kp_c(counter_i))
Kd=double(Kd_c(counter_i))
Ki=double(Ki_c)
jeff= double(jeff_c(counter_i))
q=q_c(counter_i)
set_param('Motor_load_and_controller_PID','StopTime', [Simulationtime])
sim('Motor_load_and_controller_PID')
output_q2=simout;
output_q2.data(20)
plot(output_q2.time, output_q2.data,'-x')


counter_i= 3
TL=double(T_L3);
Kp=double(Kp_c(counter_i))
Kd=double(Kd_c(counter_i))
Ki=double(Ki_c)
jeff= double(jeff_c(counter_i))
q=q_c(counter_i)
set_param('Motor_load_and_controller_PID','StopTime', [Simulationtime])
sim('Motor_load_and_controller_PID')
output_q3=simout;
output_q3.data(20)
plot(output_q3.time, output_q3.data,'-r')

font = 18
counter_i= 4
TL=double(T_L4);
Kp=double(Kp_c(counter_i))
Kd=double(Kd_c(counter_i))
Ki=double(Ki_c)
jeff= double(jeff_c(counter_i))
q=q_c(counter_i)
set_param('Motor_load_and_controller_PID','StopTime', [Simulationtime])
sim('Motor_load_and_controller_PID')
output_q4=simout;
output_q4.data(20)
plot(output_q4.time, output_q4.data,'--b')


plot(simout1.time,simout1.data)
legend({'q1','q2','q3','q4','qr'},'FontSize',font)
axis([0 0.75 0 0.48])
xlabel('time - [s]','FontSize',font)
ylabel('angle - [rad] / displacement [m]','FontSize',font)
title('A4','FontSize',font)
hold off

syms s
G = 1/(n^2);
N = vpa([jeff_c(1)*s^2+f_eff*s+kT*Kp_c(1)+kT*Kd_c(1)*s+(kT*Ki_c/s) jeff_c(2)*s^2+f_eff*s+kT*Kp_c(2)+kT*Kd_c(2)*s+kT*Ki_c/s jeff_c(3)*s^2+f_eff*s+kT*Kp_c(3)+kT*Kd_c(3)*s+kT*Ki_c/s jeff_c(4)*s^2+f_eff*s+kT*Kp_c(4)+kT*Kd_c(4)*s+kT*Ki_c/s],3)
F = vpa([Kd_c(1)*s*kT+Kp_c(1)*kT+kT*Ki_c/s Kd_c(2)*s*kT+Kp_c(2)*kT+kT*Ki_c/s Kd_c(3)*s*kT+Kp_c(3)*kT+kT*Ki_c/s Kd_c(4)*s*kT+Kp_c(4)*kT+kT*Ki_c/s],3)
tf = vpa([(1/N(1))*(F(1)*q_c(1)-G*0) (1/N(2))*(F(2)*q_c(2)-G*T_L2) (1/N(3))*(F(3)*q_c(3)-G*T_L3) (1/N(4))*(F(4)*q_c(4)-G*T_L4)],3)
ss1 = limit(tf(1))
ss2 = limit(tf(2))
0.35-ss2
ss3 = limit(tf(3))
0.35-ss3
ss4 = limit(tf(4))
0.35-ss4


