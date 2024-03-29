function aq = calcQdd(u)

ax = [u(1) u(2) u(3)]';
qc = [u(4) u(5)]';
qd = [u(6) u(7)]';
J = calcJacobian(qc);


% % Robot Link variables
% % m1 = 5; r1 = 0.1; L1 = 0.15;
% m2 = 5; r2 = 0.05; L2 = 0.7;
% m3 = 5; r3 = 0.05; L3 = 0.7;
% % m4 = 4; r4 = 0.03; L4 = 0.4;
% 
% % Inertia Matrix
% % I_1 = (1/12)*m1*(3*r1^2 + L1^2);
% % I_1yy = (1/2)*m1*r1^2;
% 
% I_2 = (1/12)*m2*(3*r2^2 + L2^2);
% I_2yy = (1/2)*m2*r2^2;
% 
% I_3 = (1/12)*m3*(3*r3^2 + L3^2);
% I_3yy = (1/2)*m3*r3^2;
% % 
% % I_4 = (1/12)*m4*(3*r4^2 + L4^2);
% % I_4yy = (1/2)*m4*r4^2;
% 
% 
% % Remember the order of rigid bodies
% I1 = [I_2 0 0; 0 I_2yy 0; 0 0 I_2];
% I2 = [I_3 0 0; 0 I_3yy 0; 0 0 I_3];
% 
% 
% robot2 = SerialLink( [ Revolute('a', 0.7,'m',5,'r',[-0.35; 0; 0], 'I', I1) ...
%     Revolute('a', 0.7,'m',5,'r',[-0.35; 0; 0],  'I', I2)],'name', 'my robot');
% robot2.gravity = [0; 0; -9.8];

% Jd = robot2.jacob_dot(qc,qd);
% J = robot2.jacob0(qc);


Jd = calcJacobianDot([qc qd]);
Jd = Jd * qd;

% Jd = Jd(1:length(qd),1:length(qd));
J = J(1:2,1:2);
lambda = 0.1;
Jinv = (J' * inv(J*J'+eye(2)*lambda))';
% Jinv = inv(J);

TestSingularity = det(J(1:2,1:2));

if (TestSingularity < 1e-11) & (TestSingularity > -1e-11)
% if (TestSingularity == 0)
    aq = [0;0];
else
    aq = Jinv*(ax(1:2) - Jd(1:2));
    if(isnan(aq))
        aq = [0;0];
    end
    double(aq);
   
end

end