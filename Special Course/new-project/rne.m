syms qd1 qdd1 l1 lc1 l2 lc2 qd2 qdd2 g 
z0 = A1(1:3,3) % should just be [0;0;1], but A1 is the exact same.
z1 = A1(1:3,3)

R01 = R0;
R02 = subs(R0,q1,q2);

gload = [0; 0; g]


r1c1 = [lc1*i; 0; 0]
r2c1 = [(l1 - lc1)*i; 0; 0]
r12 = [l1*i; 0; 0]

r2c2 = [lc2*i; 0; 0]
r3c2 = [(l2 - lc2)*i; 0; 0]
r23 = [l2*i 0 0]'


% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 

% % Forward
% Link 1
b1 = (R01')*z1
w1 = (inv(R01)') * zeros(3,1) + b1 * qd1
w1d = [0; 0; qdd1];

alpha1 = (R01') * zeros(3,1) + b1*qdd1 + cross(w1,(b1*qd1))
ac1 = cross(w1d,r1c1) + cross( w1 ,cross(w1,r1c1))
ae1 = cross(w1d,r12) + cross( w1 ,cross(w1,r12))

g1 = -R01*gload

% Link 2
b2 = (R02')*z1
w2 = (inv(R02)') * w1 + b2 * qd2
w2d = [0; 0; qdd1+qdd2];

alpha2 = (R02') * alpha1 + b2*qdd2 + cross(w2,(b2*qd2))
ac2 = simplify((inv(R02)') * ac1 + cross(w2d,r2c2) + cross( w2 ,cross(w2,r2c2)))
ae2 = simplify((inv(R02)') * ae1 + cross(w2d,r23) + cross( w2 ,cross(w2,r23)))

g2 = -R02*gload




% Backward recursion
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 



f3 = zeros(3,1)
tau3 = zeros(3,1)
R03 = R02 % doesn't really matter, since the term equals to zero


f2 = R03*f3 + m2*ac2 - m2*g2
tau2 =  R03*tau3 - cross(f2,r2c2) +  cross((R03*f3) ,r2c2) + alpha2 + cross( w2, (I2*w2) )



f1 = R02*f2 + m1*ac1 - m1*g1
tau1 =  R02*tau2 - cross(f1,r1c1) +  cross((R02*f2) ,r1c1) + alpha1 + cross( w1, (I1*w1) )