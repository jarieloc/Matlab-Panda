clear all
close all
clc

l1=500;l2=600;l3=400;l4=191.03;
t1=linspace(-180,180,90)*pi/180;
t2=linspace(-90,90,90)*pi/180;
d3=linspace(-200,200,90);
t4=linspace(-180,180,90)*pi/180;
[T1,T2,D3]=ndgrid(t1,t2,d3);  % This will create matrices of 90x90x90 for each variable
xM = round((-cos(T1).*cos(T2)).*((D3 + l2 + l3 + l4)));
yM = round((-cos(T2).*sin(T1)).*(D3 + l2 + l3 + l4));
zM = round((l1 - l4.*sin(T2) - sin(T2).*(D3 + l2 + l3)));
figure
plot3(xM(:),yM(:),zM(:),'.') % This is the plot type you should be using.
grid on
% With a '.' as an argument to show only locations and not lines
% Also, (:) converts any matrix into a list of its elements in one single column.