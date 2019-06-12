clc;
clear all;
close all;

addpath results\SpiralSim;
load results\SpiralSim\sim1Vr10.mat

% Simulation Setup
uav_v     = 10;
turnrate  = 20; 
pitchrate = 20;
Radius = 50;

psi = deg2rad(0);
gam = deg2rad(pitchrate);
lam =  1;
K = 5; % Number of turns
ovfDF = {@NoVDecay};

ZOffset = 0;
A = -Radius*(tan(gam)/lam);
theta = pi;
uavZc = A * (-theta) + ZOffset;

% Plot Options
height = 100;
bPlotUAV = true;
bAnimateUAV = true;
bSaveAnimation = false;
bPlotObstacle = false;
opt.bShowSurfaces = true;
opt.bPlotVF = false;

uavColor = {[ 0    0.4470    0.7410],...
            [0.8500    0.3250    0.0980],...
            [0.9290    0.6940    0.1250], ...
            [0.4940    0.1840    0.5560],...
            [0.4660    0.6740    0.1880]};
        
%% Obstacle field initial conditions
ovfXc = 0;
ovfYc = 0;
ovfTheta = atan2(ovfYc,ovfXc);

%% Create navigational vector fields
G =  -1;      %Convergence field
H =  5;      %Circulation field
L =  0;      %Time-varying field

cVFR = VectorField('Spiral',Radius);
cVFR.G=G;
cVFR.H=H;
cVFR.L=L;

cVFR.xc=0;
cVFR.yc=0;
cVFR.zc=0;

cVFR.vel_x=0;
cVFR.vel_y=0;
cVFR.vel_z=0;

cVFR.psi = psi;
cVFR.gam = gam;
cVFR.lam = lam; 
cVFR.K = K;
cVFR.ZStart = uavZc;

cVFR.bUseVRel = ~true;
cVFR.bUsePathFunc = ~true;

%% Create obsticle vector fields
clear avoidVF;
avoidVF = {};
ovfOpt = {};

[avoidVF, ovfOpt] = makeOVF(ovfXc, ovfYc, 1, ovfTheta,...
    ovfDF{1}, 'Obstacle 1', avoidVF, ovfOpt);

opt.bCustomRange = 50; 
opt.bShowCircle = ~true;
opt.bPlotQuiverNorm = true;
opt.DecayFunc = @NoVDecay;
opt.CustomNumberOfPoints=35;
opt.CustomCylinderHeight = height;
opt.bNormVFVectors = ~true;
opt.Color = [0 0 1];

RET_VF= cVFR.PlotFieldAroundRadius(gca,opt);

if bPlotObstacle == true
    plot_h_vf = RET_VF.H;
    for iii=1:length(avoidVF)
        ovfOpt{iii}.bPlotQuiverNorm = true;
        ovfOpt{iii}.bPlotVF = true;
        ovfOpt{iii}.bShowCircle=true;
        ovfOpt{iii}.bCustomRange = avoidVF{iii}.plotrange;
        ovfOpt{iii}.bCustomCenter = avoidVF{iii}.plotcenter;
        ovfOpt{iii}.bCustomCircleRadiusPlot = Radius;
        ovfOpt{iii}.bCustomCylinderHeight  = 30;
        ovfOpt{iii}.bNormVFVectors = true;
        ovfOpt{iii}.Color = [1 0 0];
        RET_temp = avoidVF{iii}.VF.PlotFieldAroundRadius(gca,ovfOpt{iii});
        plot_h_avoid = RET_temp.H;
    end
end

uav_v = 10;
dt = 1/(uav_v*30);
t_list=0:dt:60;
points = length(uavData.position{1,1}(:,1));

hold on;
axis equal;
xlim([-Radius*1.5 Radius*1.5]);
ylim([-Radius*1.5 Radius*1.5]);
zlim([-height*K height]);
if bPlotUAV == true
    zlim([min(uavData.position{1,1}(:,3)) max(uavData.position{1,1}(:,3))])
end

set(gca, 'FontSize', 18)
grid on;
xlabel('X-Position [-]', 'FontSize', 14);
ylabel('Y-Position [-]', 'FontSize', 14);
zlabel('Z-Position [-]', 'FontSize', 14);
view([45 25 45])

for i = 1:points
    uavX(i) = uavData.position{1,1}(i,1);
    uavY(i) = uavData.position{1,1}(i,2);
    uavZ(i) = uavData.position{1,1}(i,3);      
end

skip = 100;
XPlot = uavX(1:skip:end,1:skip:end);
YPlot = uavY(1:skip:end,1:skip:end);
ZPlot = uavZ(1:skip:end,1:skip:end);

    if bAnimateUAV == true && bSaveAnimation == true
        vidfile = VideoWriter('Descent3','MPEG-4');
        open(vidfile);
        for k = 1:length(XPlot)
            scatter3(XPlot(k),YPlot(k),ZPlot(k),...
                'MarkerEdgeColor','k',...
                'MarkerFaceColor','g',...
                'SizeData', 50);
            drawnow limitrate
            pause(dt)
            frame(k) = getframe(gcf);
        end
        writeVideo(vidfile,frame)
        close(vidfile)
    
    elseif  bAnimateUAV == true
       for k = 1:length(XPlot)
        scatter3(XPlot(k),YPlot(k),ZPlot(k),...
            'MarkerEdgeColor','k',...
            'MarkerFaceColor','g',...
            'SizeData', 50);
        drawnow limitrate
        pause(dt)
        frame(k) = getframe(gcf);
       end
       
    end


if bPlotUAV == true
    for i = 1
        uavPlot{i} = scatter3(uavData.position{i}(:,1),uavData.position{i}(:,2),uavData.position{i}(:,3),...
                    'MarkerFaceColor',uavColor{i},'MarkerEdgeColor',uavColor{i},...
                    'LineWidth',1);
    end
end
  

%% Decay functions go here
function G = VInvExp(rrin)
    G = 0.5.^(rrin);
end

function G = VTanh(rrin)
    rrt = 2.*pi.*(1 - rrin);
    G = 0.5.*(tanh(rrt)+1);
end

function G = VLin(rrin)
    G = (-0.5).*rrin + 1;
    G(G < 0) = 0;
end

function G = VGauss(rrin)
    G = 0.5.^((rrin).^2);
end

function G = VInvSq(rrin)
    G = 0.5.*(1./rrin).^2;
end

function G = NoVDecay(rrin)
    G = ones(1,length(rrin));
end