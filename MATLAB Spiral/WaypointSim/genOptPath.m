function optPath = genOptPath(uav,obstR,obstX,obstY,d_theta_R)

turnRate = uav.turnrate;
turnR = uav.turn_radius;
obstR = obstR;

y = turnR;
X = obstX - sqrt((turnR+obstR)^2-(y-obstY)^2);
xs = linspace(-(obstR+turnR)*1.5,X,10);
ys = zeros(1,length(xs));

%Angles

d_theta_R = deg2rad(d_theta_R);

d_theta_r = d_theta_R * obstR / uav.turn_radius;
L = d_theta_r*uav.turn_radius;


theta = asin((y-obstY)/(obstR+turnR));
beta = 3*pi/2:d_theta_r:2*pi-theta;

x1 = X+turnR*cos(beta(end));
y1 = y+turnR*sin(beta(end));
funct = @(gamma) fun(gamma,x1,y1,L,obstR,obstX,obstY,turnR,'g');
gamma_start = fsolve(funct,pi-theta);

gamma = gamma_start:-d_theta_R:theta;
x1 = obstX + obstR*cos(gamma(end));
y1 = obstY + obstR*sin(gamma(end));

funct = @(zeta) fun(zeta,x1,y1,L,obstR,obstX,obstY,turnR,'z');
zeta_start = fsolve(funct,pi+theta);
zeta = zeta_start:d_theta_r:3*pi/2;


%     optPathX = [xs,X+turnR*cos(beta),obstX + obstR*cos(gamma),-X+turnR*cos(zeta), -xs];
%     optPathY = [ys,y+turnR*sin(beta),obstY + obstR*sin(gamma), y+turnR*sin(zeta),  ys];

optPathX = [X+turnR*cos(beta),obstX + obstR*cos(gamma),-X+turnR*cos(zeta)];
optPathY = [y+turnR*sin(beta),obstY + obstR*sin(gamma), y+turnR*sin(zeta)];

optPath = [optPathX',optPathY'];

end

