clear all
close all
clc

syms x y z m r h

% For link 1
p = m/(pi*r^2*h)
I1xx = p*(y^2 + z^2)
l1 = int(I1xx,x,-sqrt(r^2-z^2),sqrt(r^2-z^2))
l2 = int(l1,y,(-h/2),(h/2))
l3 = int(l2,z,(-r),(r))


p = m/(pi*r^2*h)
I1yy = p*(x^2 + z^2)
l1 = int(I1yy,x,-sqrt(r^2-z^2),sqrt(r^2-z^2))
l2 = int(l1,y,(-h/2),(h/2))
l3 = int(l2,z,(-r),(r))


% For link 2 and 3 (similarly derived)
syms b

p = m/(b*b*h)
I2xx = p*(x^2 + z^2)
l1 = int(I2xx,x,-b/2,b/2)
l2 = int(l1,y,(-b/2),(b/2))
l3 = int(l2,z,(-h/2),(h/2))

p = m/(b*b*h)
I2zz = p*(x^2 + y^2)
l1 = int(I2zz,x,-b/2,b/2)
l2 = int(l1,y,(-b/2),(b/2))
l3 = int(l2,z,(-h/2),(h/2))

% For link 4
syms A
p = m/(A*h)
I4xx = p*x^2
l1 = A*int(I4xx,x,-h/2,h/2)

%%
clear all
close all
clc

syms m1 r1 L1 m2 b2 L2 m3 b3 L3 m4 a4

d1 = digits
d1 = digits(32)


I_1 = (1/12)*m1*(3*r1^2 + L1^2);
I_1yy = (1/2)*m1*r1^2;

I_2 = (1/12)*m2*(b2^2 + L2^2);
I_2zz = (1/6)*m2*(b2^2);

I_3 = (1/12)*m3*(b3^2 + L3^2);
I_3yy = (1/6)*m3*(b3^2);

I_4 = (1/12)*m4*a4^2;


I_1 = double(subs(I_1,[m1,r1,L1],[5, 0.04, 0.7]))
I_1yy = double(subs(I_1yy,[m1,r1],[5, 0.04]))

I_2 = double(subs(I_2,[m2,b2,L2],[8, 0.2, 1.68]))
I_2zz = double(subs(I_2zz,[m2,b2],[8, 0.2]))

I_3 = double(subs(I_3,[m3,b3,L3],[5, 0.15, 1.7]))
I_3yy = double(subs(I_3yy,[m3,b3],[5, 0.15]))

I_4 = double(subs(I_4,[m4,a4],[2, 1.10]))