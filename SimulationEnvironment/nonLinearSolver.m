%Testing non-linear systems solver

clc
clear
close all

xs = -10:0.5:10;
ys = xs;

figure
hold on
for i=1:length(xs)
    for j = 1:length(ys)
        X = [xs(i),ys(j)];
        F = VF(X);
        US(i,j) = F(1);
        VS(i,j) = F(2);
        XS(i,j) = xs(i);
        YS(i,j) = ys(j);
        
        mag(i,j) = sqrt(US(i,j)^2+VS(i,j)^2);
        
        US(i,j) = US(i,j)/mag(i,j);
        VS(i,j) = VS(i,j)/mag(i,j);
    end
end
quiver(XS,YS,US,VS);
axis equal







%Circular field
options = optimoptions('fsolve','Display','off');
R = .5:.5:5;
for j = 1:length(R)
    ops.m = 10;
    ops.n = 10;
    ops.xlimit = 10;
    ops.ylimit = 10;
    ops.r = R(j);
    ops.d_theta = 1;
    XYS = icPoints('circle',ops);
    grid on
    axis equal
    
    
    
    fun = @VF;
    
    location = cell(1,length(XYS));
    gradMag  = cell(1,length(XYS));
    solverFlag = cell(1,length(XYS));
    
    
    tic
    parfor i =1:length(XYS)
        X0 = [XYS(1,i),XYS(2,i)];
        [location{i},gradMag{i},solverFlag{i}] = fsolve(fun,X0,options);
    end
    1/toc
    
    
    figure
    hold on
    quiver(XS,YS,US,VS);
    axis equal
    
    for i =1:length(XYS)
        x0 = XYS(1,i);
        y0 = XYS(2,i);
        x = location{i};
        if solverFlag{i} == -2
            p1 = plot(x0,y0,'ro','markersize',7);
            p2 = plot(x(1),x(2),'r.','markersize',30);
            p3 = plot([x(1),x0],[x(2),y0],'r--','markersize',15);
        elseif solverFlag{i} ==1
            p4 = plot(x0,y0,'ko','markersize',7);
            p5 =plot(x(1),x(2),'k.','markersize',45);
            p6 =plot([x(1),x0],[x(2),y0],'k--','markersize',15);
        end
        
    end
end


%Grid
options = optimoptions('fsolve','Display','off');
ns = 3:1:10;
for j = 1:length(ns)
    ops.m = ns(j);
    ops.n = ns(j);
    ops.xlimit = 10;
    ops.ylimit = 10;
    XYS = icPoints('grid',ops);
    grid on
    axis equal
    
    
    
    fun = @VF;
    
    location = cell(1,length(XYS));
    gradMag  = cell(1,length(XYS));
    solverFlag = cell(1,length(XYS));
    
    
    tic
    parfor i =1:length(XYS)
        X0 = [XYS(1,i),XYS(2,i)];
        if X0(1) ~= 0 || X0(2) ~=0
            [location{i},gradMag{i},solverFlag{i}] = fsolve(fun,X0,options);
        end
    end
    1/toc
    
    
    figure
    hold on
    quiver(XS,YS,US,VS);
    axis equal
    
    for i =1:length(XYS)
        x0 = XYS(1,i);
        y0 = XYS(2,i);
        x = location{i};
        
        if isempty(x0)|| isempty(y0)|| isempty(x)
%             disp('igorning IC');
        else
        if solverFlag{i} == -2
            p1 = plot(x0,y0,'ro','markersize',7);
            p2 = plot(x(1),x(2),'r.','markersize',30);
            p3 = plot([x(1),x0],[x(2),y0],'r--','markersize',15);
        elseif solverFlag{i} ==1
            p4 = plot(x0,y0,'ko','markersize',7);
            p5 =plot(x(1),x(2),'k.','markersize',45);
            p6 =plot([x(1),x0],[x(2),y0],'k--','markersize',15);
        end
        end
        
    end
end






function F = VF(X)
%Compute values of each vector component
x = X(1);
y = X(2);
%Constants
theta = deg2rad(90);
a = cos(theta);
b = sin(theta);

xc = 0;
yc = 0;
r = 0.1;

decayR = 5;


UG = -(a*x+b*y)*a+b;
VG = -(a*x+b*y)*b-a;
magG = sqrt(UG^2+VG^2);


UO = 2*(x-xc)*((x-xc)^2+(y-yc)^2-r^2);%+2*(y-yc);
VO = 2*(y-yc)*((x-xc)^2+(y-yc)^2-r^2);%-2*(x-xc);
magO = sqrt(UO^2+VO^2);


ug = UG/magG;
vg = VG/magG;

uo = UO/magO;
vo = VO/magO;

r = sqrt(x^2+y^2);
p = -(tanh(2*pi*r/decayR-pi))+1;

F(1) = ug+p*uo;
F(2) = vg+p*vo;

% mag = sqrt(F(1)^2+F(2)^2);
% F(1) = F(1)/mag;
% F(2) = F(2)/mag;
end