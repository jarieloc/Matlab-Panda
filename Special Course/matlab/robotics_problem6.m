clear all
close all
clc

T1 = [0 0 0 0;0 0 0 -0.5; -1 0 0 0; 0 0 0 1]; 
T2 = [0 0 0 0.35;0 0 0 0.5; -1 0 0 0; 0 0 0 1]; 
T3 = [0 0 0 0.7;0 0 0 -0.5; -1 0 0 0; 0 0 0 1]; 
T4 = [0 0 0 1.05 ;0 0 0 0.5; -1 0 0 0; 0 0 0 1]; 
T5 = [0 0 0 1.4 ;0 0 0 -0.5; -1 0 0 0; 0 0 0 1]; 

Ttask = [1 0 0 1.5;0 1 0 0; 0 0 1 0.2; 0 0 0 1]; 

a_4 = 1.10; 
d_1 = 1.6; 

TN = {T1 T2 T3 T4 T5}; collect = []; tp_cell = []

for i = 1:5
    collect = Ttask*TN{1,i};
    tp_cell{i} = collect;
end

q1 = []; q2 = []; q3 = []; q4 = []; tp = []

for i = 1:5
    tp = tp_cell{i};
    q1(i) =  atan2( tp(2,4), tp(1,4));
    q2(i) =  atan2( sin(q1(i))*(tp(2,4)-a_4*tp(2,1))+cos(q1(i))*(tp(1,4)-a_4*tp(1,1)),tp(3,4)-d_1-a_4*tp(3,1));
    q3(i) = (sin(q1(i))*(tp(2,4)-a_4*tp(2,1))+cos(q1(i))*(tp(1,4)-a_4*tp(1,1)))/sin(q2(i));
    q4(i) = atan2( tp(3,1), ((-sin(q1(i))* tp(2,4) - cos(q1(i)) * tp(1,4) + q3(i) * sin(q2(i)))/a_4) ) - q2(i)+2*pi;
end

T = [q1;q2;q3;q4]

% Duration is 2 seconds
t0 = 0;
tf = 2;

syms q0; 
v0 = 0; 
a0 = 0; 
syms  qf; 
vf = 0; 
af = 0; 

a_mat_1 = inv([1 t0 t0^2 t0^3 t0^4 t0^5; 
               0 1 2*t0 3*t0^2 4*t0^3 5*t0^4; 
               0 0 2 6*t0 12*t0^2 20*t0^3; 
               1 tf tf^2 tf^3 tf^4 tf^5;
               0 1 2*tf 3*tf^2 4*tf^3 5*tf^4; 
               0 0 2 6*tf 12*tf^2 20*tf^3]); 

traj_vec = [q0; v0; a0; qf; vf; af]; 

segments = []; segments_collect = []; 

for i = 1:4
    q1seg = double(subs(a_mat_1 * traj_vec, [q0,qf], [q1(i), q1(i+1)]));
    q2seg = double(subs(a_mat_1 * traj_vec, [q0,qf], [q2(i), q2(i+1)]));
    q3seg = double(subs(a_mat_1 * traj_vec, [q0,qf], [q3(i), q3(i+1)]));
    q4seg = double(subs(a_mat_1 * traj_vec, [q0,qf], [q4(i), q4(i+1)]));

    segments = [q1seg'; q2seg'; q3seg'; q4seg']; 
    segments_collect{i} = segments; 
end

seg1 = [];seg2 = [];seg3 = [];seg4 = [];

seg1 = segments_collect{1};
seg2 = segments_collect{2};
seg3 = segments_collect{3};
seg4 = segments_collect{4};

syms t
seg1 = seg1 * [1;t;t^2;t^3;t^4;t^5];
seg2 = seg2 * [1;t;t^2;t^3;t^4;t^5];
seg3 = seg3 * [1;t;t^2;t^3;t^4;t^5];
seg4 = seg4 * [1;t;t^2;t^3;t^4;t^5];

tv = linspace(0,2,100);

seg1 = subs(seg1,t,tv);
seg2 = subs(seg2,t,tv);
seg3 = subs(seg3,t,tv);
seg4 = subs(seg4,t,tv);

GM_jp = [seg1 seg2 seg3 seg4]

clear q1 q2 q3 q4 d3
syms q1 q2 q3 q4 d1 d3 a4 p

A1=[cos(q1) 0 -sin(q1) 0 ; sin(q1) 0 cos(q1) 0 ; 0 -1 0 d1 ; 0 0 0 1]
A2=[cos(q2) 0 sin(q2) 0 ; sin(q2) 0 -cos(q2) 0 ; 0 1 0 0 ; 0 0 0 1]
A3=[-1 0 0 0 ; 0 0 1 0 ; 0 1 0 d3 ; 0 0 0 1]
A4 = [cos(q4) 0 sin(q4) a4*cos(q4) ; sin(q4) 0 -cos(q4) a4*sin(q4) ; 0 1 0 0 ; 0 0 0 1]

T04= simplify(A1*A2*A3*A4)
T02=A1*A2
T03=A1*A2*A3

J1 = [cross([0 0 1]',T04(1:3,4)); [0 0 1]']
J2 = [cross(A1(1:3,3),T04(1:3,4)-A1(1:3,4)) ; A1(1:3,3)]
J3 = [T02(1:3,3) ; [0 0 0]']
J4 = [cross(T03(1:3,3),T04(1:3,4)-T03(1:3,4)) ; T03(1:3,3)]
J = simplify([J1 J2 J3 J4])
rank(J)

JS = []

for i = 1:5
    JS{i} = double(subs(J, [q1, q2, d3, q4, a4], [T(1,i), T(2,i), T(3,i), T(4,i) 1.10]));
end

rank(JS{1})
pseg_xy = []

JS_plot = [];
JS_plotcollect = [];

J_cell = {J1,J2,J3,J4};
J_sim = [];
j = 0;
js = 0
js1 = 0
js2 = 0
js3 = 0
js4 = 0

JS_plots = [];
j_ranks = [];


for i = 1:(length(tv)*4)
    JS_plot = (subs(J, [q1, q2, d3, q4, a4], [GM_jp(1,i), GM_jp(2,i), GM_jp(3,i), GM_jp(4,i), 1.10]));
    k = rank(JS_plot);
    if k < 4
        js = js+1;
    end
end
j_ranks = js;

%%
clear all
close all
clc

T1 = [0 0 0 0;0 0 0 -0.5; -1 0 0 0; 0 0 0 1]; 
T2 = [0 0 0 0.35;0 0 0 0.5; -1 0 0 0; 0 0 0 1]; 
T3 = [0 0 0 0.7;0 0 0 -0.5; -1 0 0 0; 0 0 0 1]; 
T4 = [0 0 0 1.05 ;0 0 0 0.5; -1 0 0 0; 0 0 0 1]; 
T5 = [0 0 0 1.4 ;0 0 0 -0.5; -1 0 0 0; 0 0 0 1]; 

Ttask = [1 0 0 1.5;0 1 0 0; 0 0 1 0.2; 0 0 0 1]; 

a_4 = 1.10; 
d_1 = 1.6; 

TN = {T1 T2 T3 T4 T5}; collect = []; tp_cell = []

for i = 1:5
    collect = Ttask*TN{1,i};
    tp_cell{i} = collect;
end

q1 = []; q2 = []; q3 = []; q4 = []; tp = []

for i = 1:5
    tp = tp_cell{i};
    q1(i) =  atan2( tp(2,4), tp(1,4));
    q2(i) =  atan2( sin(q1(i))*(tp(2,4)-a_4*tp(2,1))+cos(q1(i))*(tp(1,4)-a_4*tp(1,1)),tp(3,4)-d_1-a_4*tp(3,1));
    q3(i) = (sin(q1(i))*(tp(2,4)-a_4*tp(2,1))+cos(q1(i))*(tp(1,4)-a_4*tp(1,1)))/sin(q2(i));
    q4(i) = atan2( tp(3,1), ((-sin(q1(i))* tp(2,4) - cos(q1(i)) * tp(1,4) + q3(i) * sin(q2(i)))/a_4) ) - q2(i)+2*pi;
end

T = [q1;q2;q3;q4]

% Duration is 2 seconds
t0 = 0;
tf = 2;

syms q0; 
v0 = 0; 
a0 = 0; 
syms  qf; 
vf = 0; 
af = 0; 

a_mat_1 = inv([1 t0 t0^2 t0^3 t0^4 t0^5; 
               0 1 2*t0 3*t0^2 4*t0^3 5*t0^4; 
               0 0 2 6*t0 12*t0^2 20*t0^3; 
               1 tf tf^2 tf^3 tf^4 tf^5;
               0 1 2*tf 3*tf^2 4*tf^3 5*tf^4; 
               0 0 2 6*tf 12*tf^2 20*tf^3]); 

traj_vec = [q0; v0; a0; qf; vf; af]; 

segments = []; segments_collect = []; 

for i = 1:4
    q1seg = double(subs(a_mat_1 * traj_vec, [q0,qf], [q1(i), q1(i+1)]));
    q2seg = double(subs(a_mat_1 * traj_vec, [q0,qf], [q2(i), q2(i+1)]));
    q3seg = double(subs(a_mat_1 * traj_vec, [q0,qf], [q3(i), q3(i+1)]));
    q4seg = double(subs(a_mat_1 * traj_vec, [q0,qf], [q4(i), q4(i+1)]));

    segments = [q1seg'; q2seg'; q3seg'; q4seg']; 
    segments_collect{i} = segments; 
end

seg1 = [];seg2 = [];seg3 = [];seg4 = [];

seg1 = segments_collect{1}
seg2 = segments_collect{2}
seg3 = segments_collect{3}
seg4 = segments_collect{4}


syms p0 pf
cart_vec = [p0; v0; a0; pf; vf; af]

pseg = []
pseg_collect = []

for i = 1:4
    tp = TN{i};
    tp2 = TN{i+1}
    pseg = []
    px = double(subs(a_mat_1 * cart_vec, [p0,pf], [tp(1,4), tp2(1,4)]))
    py = double(subs(a_mat_1 * cart_vec, [p0,pf], [tp(2,4), tp2(2,4)]))
    pseg = [px';py']
    pseg_collect{i} = pseg
end



pseg1 = pseg_collect{1}
pseg2 = pseg_collect{2}
pseg3 = pseg_collect{3}
pseg4 = pseg_collect{4}

clear q1 q2 q3 q4 d3
syms q1 q2 q3 q4 d1 d3 a4 p

A1=[cos(q1) 0 -sin(q1) 0 ; sin(q1) 0 cos(q1) 0 ; 0 -1 0 d1 ; 0 0 0 1]
A2=[cos(q2) 0 sin(q2) 0 ; sin(q2) 0 -cos(q2) 0 ; 0 1 0 0 ; 0 0 0 1]
A3=[-1 0 0 0 ; 0 0 1 0 ; 0 1 0 d3 ; 0 0 0 1]
A4 = [cos(q4) 0 sin(q4) a4*cos(q4) ; sin(q4) 0 -cos(q4) a4*sin(q4) ; 0 1 0 0 ; 0 0 0 1]

T04= simplify(A1*A2*A3*A4)
T02=A1*A2
T03=A1*A2*A3

J1 = [cross([0 0 1]',T04(1:3,4)); [0 0 1]']
J2 = [cross(A1(1:3,3),T04(1:3,4)-A1(1:3,4)) ; A1(1:3,3)]
J3 = [T02(1:3,3) ; [0 0 0]']
J4 = [cross(T03(1:3,3),T04(1:3,4)-T03(1:3,4)) ; T03(1:3,3)]
J = simplify([J1 J2 J3 J4])
rank(J)

JS = []

for i = 1:5
    JS{i} = double(subs(J, [q1, q2, d3, q4, a4], [T(1,i), T(2,i), T(3,i), T(4,i) 1.10]));
end

rank(JS{1})
pseg_xy = []

syms t
for i = 1:4
    pseg = pseg_collect{i}
    psegx = pseg(1,1) + pseg(1,2)*t + pseg(1,3)*t.^2 + pseg(1,4)*t.^3 + pseg(1,5)*t.^4 + pseg(1,6)*t.^5;
    psegy = pseg(2,1) + pseg(2,2)*t + pseg(2,3)*t.^2 + pseg(2,4)*t.^3 + pseg(2,5)*t.^4 + pseg(2,6)*t.^5;
    pseg_xy{i} = [psegx;psegy]
end

pseg_xy{1}
pseg_xy{2}
pseg_xy{3}
pseg_xy{4}


d_1 = 1.6;
a_4 = 1.10;

q1ss = []
q2ss = []
q3ss = []
q4ss = []
QSS = []

for i = 1:4
    seg = pseg_xy{i};
    segX = seg(1,:);
    segY = seg(2,:);
    q1ss{i} = atan2( segY, segX );
    q2ss{i} = atan2( cos(q1ss{i})* segX + sin(q1ss{i}) * segY, 0.2- d_1 - a_4*(-1) );
    q3ss{i} = (cos(q1ss{i}) * segX + sin(q1ss{i}) * segY)/sin(q2ss{i});
    q4ss{i} = atan2(-1,(-sin(q1ss{i}) * segY-cos(q1ss{i}) * segX + q3ss{i} * sin(q1ss{i}) / a_4)) - q2ss{i} + 2*pi;
end

test = linspace(0,2)
qss_temp = []
QSS = []

for i = 1:4
    qss_temp1 = q1ss{i};
    qss_temp2 = q2ss{i};
    qss_temp3 = q3ss{i};
    qss_temp4 = q4ss{i};    
    QSS{i} = subs([qss_temp1; qss_temp2; qss_temp3; qss_temp4],t,test);
end
QSS1 = QSS{1};
QSS2 = QSS{2};
QSS3 = QSS{3};
QSS4 = QSS{4};

JS_plot = [];
JS_plotcollect = [];

J_cell = {J1,J2,J3,J4};
J_sim = [];
j = 0;
js = 0
js1 = 0
js2 = 0
js3 = 0
js4 = 0

JS_plots = [];
j_ranks = [];

for j = 1:4
    QSS_Temp = QSS{j};
    for i = 1:length(linspace(0,2))
        JS_plot = (subs(J, [q1, q2, d3, q4, a4], [QSS_Temp(1,i), QSS_Temp(2,i), QSS_Temp(3,i), QSS_Temp(4,i), 1.10]));
        k = rank(JS_plot);
        if k < 4
            js = js+1;
        end
    end
    j_ranks(j) = js;
end
j_ranks



