clear all
close all
clc


q1 = [linspace(0,0.35,10) linspace(0.35,0,10)];
q2 = [linspace(0,0.35,10) linspace(0.35,0,10)];

% q1d = linspace(0,1,20);
% q2d = linspace(0,1,20);

% q1d = zeros(1,20)
% q2d = zeros(1,20)

q1d = [0 diff(q1)];
q2d = [0 diff(q2)];

q1dd = [0 diff(q1d)];
q2dd = [0 diff(q2d)];



% Loop setup variables
n = length(q1); collect = []; T = eye(4); Tcol = []; R = [];

% Robot DH variables
d_1 = 0; a1 = 0.7; a2 = 0.7;
alphaT =[0 0]; d = [d_1 0]; a = [a1 a2];

% Robot Link variables
m1 = 5; r1 = 0.1; L1 = 0.15;
m2 = 5; r2 = 0.05; L2 = 0.7;
m3 = 5; r3 = 0.05; L3 = 0.7;
m4 = 4; r4 = 0.03; L4 = 0.4;

lc1 = (L2/2); lc2 = (L3/2);
l1 = L2; l2 = L3;


% Inertia Matrix
I_1 = (1/12)*m1*(3*r1^2 + L1^2);
I_1yy = (1/2)*m1*r1^2;

I_2 = (1/12)*m2*(3*r2^2 + L2^2);
I_2yy = (1/2)*m2*r2^2;

I_3 = (1/12)*m3*(3*r3^2 + L3^2);
I_3yy = (1/2)*m3*r3^2;

I_4 = (1/12)*m4*(3*r4^2 + L4^2);
I_4yy = (1/2)*m4*r4^2;


% Remember the order of rigid bodies
I1 = [I_2 0 0; 0 I_2yy 0; 0 0 I_2];
I2 = [I_3 0 0; 0 I_3yy 0; 0 0 I_3];
I = I1;
I(:,:,2) = I2;

vec1 = []; temp1 = [];
vec2 = []; temp2 = [];
Q = []; t1 = []; t2 = [];



r1c1 = [lc1; 0; 0]; r2c1 = [(l1 -lc1); 0; 0]; r12 = [l1; 0; 0];
r2c2 = [lc2; 0; 0]; r3c2 = [(l2 -lc2); 0; 0]; r23 = [l2 0 0]';

m = [m1; m2]; 
re = [r12, r23]; 
rc = [r1c1, r2c1];
rc(:,:,2) = [r2c2, r3c2]

g = [0; 0; -9.8]; b = []; w = []; wd = []; alpha = [];
ae = []; ac = []; gload = []; f = []; tau = []; z = [];
f = zeros(3,2);
tau = zeros(3,2);

for j = 1:n
theta = [q1(j) q2(j)]';
qd = [q1d(j) q2d(j)]';
qdd = [q1dd(j) q2dd(j)]';

% The algorithm begins:
for i = 1:length(theta)
%%% % Forward

T = calc_transformation(i-1, i, alphaT, a, theta, d);
R = T(1:3,1:3);
z = T(1:3,3);
z0 = [0;0;1];
b = (R*z0)'* z;
if (i == 1)
    T = calc_transformation(i-1, i, alphaT, a, theta, d);
    z1 = T(1:3,3);
    
    w = [w (((R*z0)') * zeros(3,1) + b * qd(i))]; 
    wd = [wd (((R)') * zeros(3,1) + b * qd(i))]; 
    alpha = [alpha ((R * z0)' * zeros(3,1) + b * qdd(i)  + cross(w(i),(b*qd(i))) )];

    
    ac = [ac (cross(wd(:,i),rc(:,i,i)) + cross( w(:,i) ,cross(w(:,i),rc(:,i,i))))];
    ae = [ae (cross(wd(:,i),re(:,i)) + cross(w(:,i) ,cross(w(:,i),re(:,i))))];

else
    w(:,i) = ((R*z)') * w(:,i-1) + b * qd(i);
    wd(:,i) = I(:,:,i) * wd(:,i-1) + z1 * qdd(i) + cross((I(:,:,i)*w(:,i)), (z0*qd(i)));
    alpha(:,i) = (R * z)' * alpha(:,i-1) + b*qdd(i)  + cross(w(:,i),(b*qd(i) ));
    
    ae(:,i) = ((R)') * ae(:,i-1) + cross(wd(:,i),re(:,i)) +  cross( w(:,i) ,cross(w(:,i),re(:,i)));
    ac(:,i) = ((R)') * ac(:,i-1) + cross(wd(:,i),rc(:,i)) +  cross( w(:,i) ,cross(w(:,i),rc(:,i)));
end
gload = [gload -R*g];

end




% Backward
for i=length(theta):-1:1
    T = calc_transformation(i-1, i, alphaT, a, theta, d);
    R = T(1:3,1:3);
    z = T(1:3,3);
    b = (R')* z;

    if (i == length(theta));
        T = calc_transformation(i-1, i, alphaT, a, theta, d);
        z2 = T(1:3,3);
        R2 = T(1:3,1:3);
        
        f3 = zeros(3,1);
        tau3 = zeros(3,1);
        R03 = zeros(3,3);
        rc3 = zeros(3,1);
        f(:,i) =  m(i)*ac(:,i) - m(i) * gload(:,i)
        tau(:,i) = alpha(:,i) + cross(w(:,i),(I(:,:,i)*w(:,i))) - cross(f(:,i),rc(:,i));
        
        
%         tau(:,i) =  R03*tau3 - cross(f(:,i),rc(:,i)) +  cross((R03*f3) ,rc3)...
%             + alpha(:,i) + cross( w(:,i), (I(:,:,i)*w(:,i)) );
    else
%         j = ((n+1)-i);        
        f(:,i) = double(R2*f(:,i+1) + m(i)*ac(:,i) - m(i)*gload(:,i));
        tau(:,i) =  (R*tau(:,i+1)) - cross(f(:,i),rc(:,i)) +...
            cross((R*f(:,i+1)) ,rc(:,i+1)) + alpha(:,i) + cross( w(:,i), (I(:,:,i)*w(:,i)) );
    end
    for i = 1:2
        Q = [Q (tau(:,i))]; 
        t1 = [t1 tau(:,1)];
        t2 = [t2 tau(:,2)];
%         Q(i,j) = tau(:,i)'*R'* z
    end
end

% tau;
% for i = 1:2
% %     Q = [Q (tau(:,i))]; 
% %     t1 = [t1 tau(:,1)];
% %     t2 = [t2 tau(:,2)];
%     Q(i,j) = tau(:,i)'*R'* [0;0;1];
% end


end
Q = Q';
t1 = t1';
t2 = t2';
% col = [t1 t2];




robot2 = SerialLink( [ Revolute('a', 0.7,'m',5,'r',[0.35; 0; 0]) Revolute('a', 0.7,'m',5,'r',[0.35; 0; 0]) ],'name', 'my robot')
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
% tres1 = double(Qout(1:3,:));
% tres2 = double(Qout(4:6,:));
% tres1 = tres1';
% tres2 = tres2';

% 
figure()
subplot(1,2,1),plot(t1(:,3)); hold on; plot(t2(:,3)); hold off; grid on;
% subplot(1,2,1),plot(t1(:,1),t1(:,2)); hold on; plot(t2(:,1),t2(:,2)); hold off; grid on;
% subplot(1,2,1),plot(t1); grid on;
legend('\tau_1 computed','\tau_2 computed')
xlabel('samples')
ylabel('[N?m]')
title('Computed RNEA')

subplot(1,2,2),plot(1:length(q1),torque(:,1),'r', 1:length(q1), torque(:,2),'b'); grid on; 
legend('\tau_1 ToolB','\tau_2 ToolB')
xlabel('samples')
ylabel('[N?m]')
title('Robotics ToolBox RNEA')