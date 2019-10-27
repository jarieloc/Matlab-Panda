clear all
close all
clc

T1 = [0 0 0 0;0 0 0 -0.5; -1 0 0 0; 0 0 0 1]
T2 = [0 0 0 0.35;0 0 0 0.5; -1 0 0 0; 0 0 0 1]
T3 = [0 0 0 0.7;0 0 0 -0.5; -1 0 0 0; 0 0 0 1]
T4 = [0 0 0 1.05 ;0 0 0 0.5; -1 0 0 0; 0 0 0 1]
T5 = [0 0 0 1.4 ;0 0 0 -0.5; -1 0 0 0; 0 0 0 1]

Ttask = [1 0 0 1.5;0 1 0 0; 0 0 1 0.2; 0 0 0 1]

a_4 = 1.10
d_1 = 1.6

TN = {T1 T2 T3 T4 T5}
collect = []
tp_cell = []

% TN is an array cell, not a matrix - but a collection of matrices.

for i = 1:5
    collect = Ttask*TN{i}
    tp_cell{i} = collect
end

q1 = [];
q2 = [];
q3 = [];
q4 = [];

tp = [];

for i = 1:5
    tp = tp_cell{i};
    q1(i) =  atan2( tp(2,4), tp(1,4));
    q2(i) =  atan2( ((tp(1,1)*a_4)-(-1*tp(1,4)))/cos(q1(i)) , -a_4*tp(3,1) - d_1 + tp(3,4) )
    q4(i) = -acos( tp(2,1) / sin(q1(i)) ) - q2(i)+2*pi;
    q3(i) = ( ( tp(1,4)/-cos(q1(i)) ) - a_4 * cos(q2(i) + q4(i)) ) / -sin(q2(i));
end



T = [q1;q2;q3;q4]



plot_collect = [];
plotdata = [];

for i = 1:5
    tp = tp_cell{i};
    plot_collect = [plot_collect tp(:,4)];
%     stem3(plot_collect(1), plot_collect(2), plot_collect(3))
end

plot_collect;

%%
plot_collect(4,:) = []

x = plot_collect(1,:)
y = plot_collect(2,:)
z = plot_collect(3,:)

q1 = rad2deg(T(1,:))
q2 = rad2deg(T(2,:))
q3 = T(3,:)
q4 = rad2deg(T(4,:))


figure()
plot3(x,y,z)
grid on
title('Knot-points')

figure()
subplot(2,2,1)
plot((q1), '--or')
grid on
ylabel('[Rad]')
xlabel('Points')
title('q_1')
xlim([0,5])

subplot(2,2,2)
plot((q2), '--or')
grid on
ylabel('[Rad]')
xlabel('Points')
xlim([0,5])
title('q_2')

subplot(2,2,3)
plot(q3, '--ob')
ylabel('[m]')
xlabel('Points')
title('q_3')
xlim([0,5])
grid on

subplot(2,2,4)
plot((q4), '--or')
ylabel('[Rad]')
xlabel('Points')
title('q_4')
xlim([0,5])
grid on