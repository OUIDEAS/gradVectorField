clc;clear;close all
format compact
r=30;
n = 100;
count = 1;
zl = 150;

theta  = linspace(-2*pi,2*pi,n);
x_list = linspace(-r,r,n);
z_list = linspace(-zl,zl,n);
y_list = linspace(-r,r,n);

figure;hold all;

for zi = 1:length(z_list)
    for th = 1:length(theta)
        x(zi,th) = r*cos(theta(th));
        y(zi,th) = r*sin(theta(th));
        z(zi,th) = z_list(zi);
        count = count + 1;
    end
end

gam  = 0.15;
psi  = 0.15; 
lam = 1; % CW vs CCW
K = 10;  % Number of turns

xc = 0;
yc = 0;
zc = 0;

A = (-r * tan(gam)) / lam;

theta = pi;
shift  =  A * (-theta)
change =  A * (theta)


for xi = 1:length(x_list)
    for yi = 1:length(y_list)
         X(xi,yi) = x_list(xi);
         Y(xi,yi) = y_list(yi);
 
         for turns = 1:K
             Z(xi,yi,turns)  = A * (atan2(Y(xi,yi),X(xi,yi)) - psi) - shift * (turns-1) * 2;
         end  
         
    end
end

for data =  1:K
    mesh(X,Y,Z(:,:,data))
end
   
% mesh(x,y,z);
% mesh(X,Y,Z)
xlabel("x");
ylabel("y");
zlabel("z");
grid on
axis equal
return;