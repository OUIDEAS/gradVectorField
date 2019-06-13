clc;
close all;
clear all;

addpath results\sim1;

%% UAV parameters
uav_v = 10;
turnrate = 20;

uavXc = -8;
uavYc = 0;
uavTheta = 0;
dt = 0.001;
t_list=0:dt:(30./uav_v);

%% Obstacle field initial conditions
ovfXc = 0;
ovfYc = 0;
ovfTheta = atan2((ovfYc - uavYc),(ovfXc - uavXc));
ovfDF = {@VTanh};    %Function
dfName = {'Hyperbolic Tangent'};
uavColor = {[ 0    0.4470    0.7410],...
            [0.8500    0.3250    0.0980],...
            [0.9290    0.6940    0.1250],...
            [0.4940    0.1840    0.5560],...
            [0.4660    0.6740    0.1880]};

%% Animation options
bShowVectorField=true;
bRunUAV=true;

for i = 1:length(ovfDF)
    %% Create UAV object
    xVUAV = VFUAV(dt);
    xVUAV = xVUAV.SetPosition([uavXc ; uavYc]);
    uo.vx = uav_v*cos(uavTheta);
    uo.vy = uav_v*sin(uavTheta);
    uo.heading = uavTheta;
    xVUAV = xVUAV.SetVelocityAndHeading(uo); clear uo;
    xVUAV.bVFControlVelocity=~true;
    xVUAV.bVFControlHeading=~true;
    xVUAV.bDubinsPathControl = true;
    xVUAV.mTurnrate = turnrate;
    xVUAV.bNormVFVectors=~true;

    %% Create navigational vector fields
    G=-1;      %Convergence field
    H=-1;      %Circulation field
    L=0;       %Time-varying field

    cVFR = CircleVectorField('Straight',2);
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

    [avoidVF, ovfOpt] = makeOVF(ovfXc, ovfYc, 2, ovfTheta,...
        ovfDF{i}, 'Obstacle 1', avoidVF, ovfOpt);

    %% Run simulation
    plot_h_avoid=[];
    RET_VF=[];
    framecount=1;
    
    for k=1:length(t_list)
        t=t_list(k);

        if(bShowVectorField)
            clear opt;
            opt.bCustomRange = 10; 
            opt.bShowCircle=~true;
            opt.bPlotQuiverNorm=true;
            opt.DecayFunc = @NoVDecay;
            opt.CustomNumberOfPoints=25;
            opt.Color = [0 0 1];
            opt.UAV = xVUAV;

            for ii=1:length(avoidVF)
                ovfOpt{ii}.bPlotQuiverNorm = true;
                ovfOpt{ii}.bShowCircle=true;
                ovfOpt{ii}.bCustomRange = avoidVF{ii}.plotrange;
                ovfOpt{ii}.bCustomCenter = avoidVF{ii}.plotcenter;
                ovfOpt{ii}.bCustomCircleRadiusPlot = avoidVF{ii}.plotradius;
                ovfOpt{ii}.Color = [1 0 0];
                ovfOpt{ii}.UAV = xVUAV;
            end
        end

        if(bRunUAV)
            pos = xVUAV.GetPosition();
            errX(k) = pos(1) - (t.*uav_v);
            errY(k) = pos(2) - 0;

            opt.DecayFunc = ovfDF{i};
            opt.oVFList = avoidVF;
            xVUAV = xVUAV.UpdateControlFromVF(cVFR,t,opt);

            if(isempty(cVFR.radFunc))
                cVFR = cVFR.UpdatePosition(t,dt);
            else
            uav_v = xVUAV.GetVelocityV();
            uavv.x = uav_v(1);
            uavv.y = uav_v(2);
            cVFR = cVFR.UpdatePosition(t,dt,uavv,opt);
            end
        end
    end

    uavData.name{i} = dfName{i};
    uavData.position{i} = xVUAV.mPositionHistory';
    uavData.error{i}(:,1) = errX;
    uavData.error{i}(:,2) = errY;
     
    clear xVUAV.mPositionHistory;
    clear errX;
    clear errY;
end

save(['results\sim1\sim1Vr' num2str(uav_v) '.mat'], 'uavData');

%% Decay functions go here

function G = VTanh(rrin)
    rrt = 2.*pi.*(1 - rrin);
    G = 0.5.*(tanh(rrt)+1);
end

function G = NoVDecay(rrin)
    G = ones(1,length(rrin));
end