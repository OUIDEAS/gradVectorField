clc;clear;close all

xq = rand(1,300);                                           % Random X-Coordinates
yq = rand(1,300);                                           % Random Y-Coordinates

centerX = -50;
centerY = 0;
x1=0 + centerX;
x2=100 + centerX;
y1=0 + centerY;
y2=50 + centerY;
xv = [x1, x2, x2, x1, x1];
yv = [y1, y1, y2, y2, y1];



[in,on] = inpolygon(xq,yq, xv,yv);                          % Logical Matrix
inon = in | on;                                             % Combine ‘in’ And ‘on’
idx = find(inon(:));                                        % Linear Indices Of ‘inon’ Points
xcoord = xq(idx);                                           % X-Coordinates Of ‘inon’ Points
ycoord = yq(idx);                                           % Y-Coordinates Of ‘inon’ Points
figure(1)
plot(xq, yq, 'bp')                                          % Plot All Points
hold on
plot(xv, yv, '-r')                                          % Plot Polygon
plot(xcoord, ycoord, 'gp')                                  % Overplot ‘inon’ Points
hold off