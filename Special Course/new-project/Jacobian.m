% Jacobian computation script
syms q1(t) q2(t) d1 lc1 lc2 d_1 a1 a2 real

% lc1 = a1/2
% lc2 = a2/2

alpha =[0 0];
d = [d_1 0];
theta = [q1(t) q2(t)];
a = [a1 a2];

% A = [cos(theta(i)) -cos(alpha(i))*sin(theta(i)) sin(alpha(i))*sin(theta(i)) a(i)*cos(theta(i));
%         sin(theta(i)) cos(alpha(i))*cos(theta(i)) -sin(alpha(i))*cos(theta(i)) a(i)*sin(theta(i));
%         0 sin(alpha(i)) cos(alpha(i)) d(i);
%         0 0 0 1]
    
collect = [];
T = eye(4);

for i = 1:2
    collect{i} = [cos(theta(i)) -cos(alpha(i))*sin(theta(i)) sin(alpha(i))*sin(theta(i)) a(i)*cos(theta(i));
        sin(theta(i)) cos(alpha(i))*cos(theta(i)) -sin(alpha(i))*cos(theta(i)) a(i)*sin(theta(i));
        0 sin(alpha(i)) cos(alpha(i)) d(i);
        0 0 0 1];   
    T = T* collect{i}
end
A1 = collect{1}
A2 = collect{2}

syms l1 l2 real
A1c = subs(A1, [a1,a2],[l1,l2]);

R1 = A1(1:3,1:3)
R2 = A2(1:3,1:3)


J1 = [cross([0 0 1]',T(1:3,4)); [0 0 1]']
J2 = [cross(A1(1:3,3),T(1:3,4)-A1(1:3,4)) ; A1(1:3,3)]


% Setup
J1_v = [cross([0 0 1]',A1(1:3,4))];
J1_w = [0 0 1]';
J2_v = [cross(A1c(1:3,3),T(1:3,4)-A1c(1:3,4))];
J2_w = [A1c(1:3,3)];


% Collect
J1v = [J1_v zeros(3,1)];
J1w = [J1_w zeros(3,1)];

% Collect
J1_v = [cross([0 0 1]',A1c(1:3,4))];
J2v = [(J1_v+J2_v) J2_v];
J2w = [(J2_v+J2_v) J2_v];

% Substitute
J1c_v = subs(J1v,[a1,a2],[lc1,lc2]);
J1c_w = subs(J1w,[a1,a2],[lc1,lc2]);
J2c_v = subs(J2v , [a1,a2],[lc1,lc2]);
J2c_w = subs(J2w , [a1,a2],[lc1,lc2]);


J = simplify([J1 J2])
rank(J);