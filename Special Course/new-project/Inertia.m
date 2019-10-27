% Inertia Computation
syms m1 r1 L1 m2 r2 L2 m3 r3 L3 m4 r4 L4 real


I_1 = (1/12)*m1*(3*r1^2 + L1^2);
I_1yy = (1/2)*m1*r1^2;

I_2 = (1/12)*m2*(3*r2^2 + L2^2);
I_2yy = (1/2)*m2*r2^2;

I_3 = (1/12)*m3*(3*r3^2 + L3^2);
I_3yy = (1/2)*m3*r3^2;

I_4 = (1/12)*m4*(3*r4^2 + L4^2);
I_4yy = (1/2)*m4*r4^2;



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
I1 = [I_1 0 0; 0 I_1yy 0; 0 0 I_1];
I2 = [I_2 0 0; 0 I_2yy 0; 0 0 I_2];
I3 = [I_3 0 0; 0 I_3yy 0; 0 0 I_3];
I4 = [I_4 0 0; 0 I_4yy 0; 0 0 I_4];


% I1m = max(max(I1))
% I2m = max(max(I2))
% I3m = max(max(I3))
% I4m = max(max(I4))
% 
% I_imax = [I1m; I2m; I3m; I4m]
% D_hat = diag(I_imax)

% syms q_d


% size(I2)
% p1 = (I2 * [eye(3) zeros(3); zeros(3) zeros(3)]

% p1 = [I2 zeros(3); zeros(3) zeros(3)]
% p2 = [I3 I3; I3 I3]

% I_col = p1 + p2
syms I2_temp I3_temp real
% I_col = [I2_temp 0; 0 0] + [I3_temp I3_temp; I3_temp I3_temp];

I_col = (J1c_w' * R1 * I2 * R1' * J1c_w + J2c_w' * R2 * I3 * R2' * J2c_w)
D = (m1*J1c_v' * J1c_v + m2*J2c_v' * J2c_v) + I_col;
Dq = simplify(D);



d11 = Dq(1,1);
d12 = Dq(1,2);
d22 = Dq(2,2);
d21 = d12;
