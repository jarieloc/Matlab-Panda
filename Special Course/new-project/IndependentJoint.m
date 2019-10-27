% Independent Joint Control Computation

run('Jacobian.m')
run('Inertia.m')
run('Coriolis.m')

q1t = linspace(0,0.35,100);
q2t = linspace(0,0.35,100);

q1d = [0 diff(q1t)];
q2d = [0 diff(q2t)];
% 
q1dd = [0 diff(q1d)];
q2dd = [0 diff(q2d)];

% q1d = zeros(1,length(q1t));
% q2d = zeros(1,length(q1t));
% 
% q1dd = zeros(1,length(q1t));
% q2dd = zeros(1,length(q1t));



% p2 = linspace(0,deg2rad(150),100)
% p3 = linspace(pi,0,100)
% p4 = linspace(deg2rad(150),deg2rad(-150),100)

J_M = 1340 * 10^(-7);

I_1 = double(subs(I_1,[m1,r1,L1],[5, 0.1, 0.15]));
I_1yy = double(subs(I_1yy,[m1,r1],[5, 0.1]));

I_2 = double(subs(I_2,[m2,r2,L2],[5, 0.05, 0.7]));
I_2yy = double(subs(I_2yy,[m2,r2],[5, 0.04]));

I_3 = double(subs(I_3,[m3,r3,L3],[5, 0.05, 0.7]));
I_3yy = double(subs(I_3yy,[m3,r3],[5, 0.04]));

I_4 = double(subs(I_4,[m4,r4,L4],[4, 0.03, 0.4]));
I_4yy = double(subs(I_4yy,[m4,r4],[4, 0.03]));



I2res = I_2 - I_2yy;
I3res = I_3 - I_3yy;

% collections = []
d11 = [];
d22 = [];

% m1 and m2 are substituted with m2 and m3
% lc1 and lc2 are half of the link length

Tl_c = [];
for i=1:length(q1t)
    Dtest = vpa(subs(Dq,[q1,q2,I2_temp,I3_temp,m1,m2,lc1,lc2,l1,l2],[q1t(i),q2t(i),I2res,I3res,5,5,(0.7/2),(0.7/2),0.7,0.7]),4);
    Tl_col = vpa(subs(torque,[q1,q2,I2_temp,I3_temp,m1,m2,lc1,lc2,l1,l2,q1_d, q2_d,q1_dd, q2_dd, g],[q1t(i),q2t(i),I2res,I3res,5,5,(0.7/2),(0.7/2),0.7,0.7, q1d(i), q2d(i), q1dd(i), q2dd(i), 9.8]),4);
%     simplify(Dtest);
    d11 = [Dtest(1,1) d11];
    d22 = [Dtest(2,2) d22];
    Tl_c = [Tl_col Tl_c];
end
% plot(collections)


d11_max = vpa(simplify(max(d11)),4);
d22_max = vpa(simplify(max(d22)),4);
n = 53;
Jeff_c = vpa((1/n^2)*[d11_max d22_max]+J_M*[1 1],4)
