% Christoph Symbols

% c111 = (1/2) * diff(d11,q1);
% c121 = (1/2) * diff(d11,q2);
% c211 = c121;
% c221 = diff(d12,q2) - ((1/2) * diff(d22,q1));
% c112 = diff(d21,q1) - ((1/2) * diff(d11,q2));
% c122 = (1/2) * diff(d22,q1);
% c212 = c122;
% c222 = (1/2) * diff(d22,q2);

c111 = (1/2) * functionalDerivative(d11,q1);
c121 = (1/2) * functionalDerivative(d11,q2);
c211 = c121;
c221 = functionalDerivative(d12,q2) - ((1/2) * functionalDerivative(d22,q1));
c112 = functionalDerivative(d21,q1) - ((1/2) * functionalDerivative(d11,q2));
c122 = (1/2) * functionalDerivative(d22,q1);
c212 = c122;
c222 = (1/2) * functionalDerivative(d22,q2);

% Coriolis Matrix

syms g q1(t) q2(t)

P1 = m1*g*d_1;
P2 = m2*g*d_1;

P = P1 + P2;

phi_1 = functionalDerivative(P,q1);
phi_2 = functionalDerivative(P,q2);

g_load = [phi_1;phi_2];
C = simplify([(c111 + c211) (c121 + c221);  (c112+c212) (c122+c222)]);

% syms q1_d q2_d q1_dd q2_dd
% q_dd = [q1_dd;q2_dd];
% q_d = [q1_d; q2_d];

torque = D*[q1_dd;q2_dd] + C*q_d + g*q_dd;
torque = simplify(torque);