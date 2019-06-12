clc;
clear all;
close all;

addpath results\sim1;
addpath figures\sim1;

load results\sim1\sim1Vr10.mat

ovfDF = {@VTanh};

uavColor = {[ 0    0.4470    0.7410],...
            [0.8500    0.3250    0.0980],...
            [0.9290    0.6940    0.1250],...
            [0.4940    0.1840    0.5560],...
            [0.4660    0.6740    0.1880]};
        
%% Obstacle field initial conditions
ovfXc = 0;
ovfYc = 0;
ovfTheta = atan2(ovfYc,ovfXc);

%% Animation options
fig1=figure; set(gcf, 'Color', 'w');    hold on;
bShowVectorField=true;
bRunUAV=true;
fig1.Position = [0 0 1200 800];

hold on;

%% Create navigational vector fields
G=-1;      %Convergence field
H=-1;      %Circulation field
L=0;       %Time-varying field

cVFR = CircleVectorField('Straight',0);
cVFR.G=G;
cVFR.H=H;
cVFR.L=L;
cVFR.xc=0;
cVFR.yc=0;
cVFR.vel_x=0;
cVFR.vel_y=0;
cVFR.bUseVRel = ~true;
cVFR.bUsePathFunc = ~true;

%% Create obsticle vector fields
clear avoidVF;
avoidVF = {};
ovfOpt = {};

[avoidVF, ovfOpt] = makeOVF(ovfXc, ovfYc, 1, ovfTheta,...
    ovfDF{1}, 'Obstacle 1', avoidVF, ovfOpt);

opt.bCustomRange = 10; 
opt.bShowCircle=~true;
opt.bPlotQuiverNorm=true;
opt.DecayFunc = @NoVDecay;
opt.CustomNumberOfPoints=25;
opt.bNormVFVectors = ~true;
opt.Color = [0 0 1];
RET_VF= cVFR.PlotFieldAroundRadius(gca,opt);
plot_h_vf = RET_VF.H;

for ii=1:length(avoidVF)
    ovfOpt{ii}.bPlotQuiverNorm = true;
    ovfOpt{ii}.bShowCircle=true;
    ovfOpt{ii}.bCustomRange = avoidVF{ii}.plotrange;
    ovfOpt{ii}.bCustomCenter = avoidVF{ii}.plotcenter;
    ovfOpt{ii}.bCustomCircleRadiusPlot = avoidVF{ii}.plotradius;
    ovfOpt{ii}.bNormVFVectors = true;
    ovfOpt{ii}.Color = [1 0 0];
    RET_temp = avoidVF{ii}.VF.PlotFieldAroundRadius(gca,ovfOpt{ii});
    plot_h_avoid = RET_temp.H;
end

for(i = 1:length(ovfDF))
    uavPlot{i} = scatter(uavData.position{i}(:,1),uavData.position{i}(:,2),...
                'MarkerFaceColor',uavColor{i},'MarkerEdgeColor',uavColor{i},...
                'LineWidth',1);
end

hold off;
axis equal;
xlim([-8 8]);
ylim([-2 4]);
set(gca, 'FontSize', 18)
grid on;
xlabel('X-Position [-]', 'FontSize', 14);
ylabel('Y-Position [-]', 'FontSize', 14);

%% Decay functions go here


function G = VTanh(rrin)
    rrt = 2.*pi.*(1 - rrin);
    G = 0.5.*(tanh(rrt)+1);
end


function G = NoVDecay(rrin)
    G = ones(1,length(rrin));
end