clc;
close all;
clear all;
format compact;

addpath results\SpiralSim;

%% UAV parameters
uav_v     = 10;
turnrate  = 20; 
pitchrate = 20;

psi = deg2rad(0);
gam = deg2rad(pitchrate);
lam = 1;
K = 5;

Radius = 50;
A = -Radius*(tan(gam)/lam);
theta = pi;

XOffset = 0;
YOffset = 0;
ZOffset = 0;

uavXc = -40;
uavYc = -20;
uavZc = A * (-theta) + ZOffset;

uavTheta = 0;
uavPitch = 0;
dt = 1/(uav_v*30);
% dt = .01;
% t_list=0:dt:(30./uav_v);
t_list=0:dt:200;

%% Obstacle field initial conditions
ovfXc = 0.0;
ovfYc = 0;0.;
ovfTheta = atan2((ovfYc - uavYc),(ovfXc - uavXc));
ovfDF = {@NoVDecay};    %Function
dfName = {'No Decay'};
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
    xVUAV = VFUAV3D(dt);
    xVUAV = xVUAV.SetPosition([uavXc ; uavYc; uavZc]);
    uo.vx = uav_v*cos(uavTheta)*cos(uavPitch);
    uo.vy = uav_v*sin(uavTheta)*cos(uavPitch);
    uo.vz = -uav_v*sin(uavPitch);
    
    uo.heading(i) = uavTheta;
    uo.pitch(i) = uavPitch;
    
    xVUAV = xVUAV.SetVelocityAndHeading(uo); clear uo;
    xVUAV.bVFControlVelocity=~true;
    xVUAV.bVFControlHeading=~true;
    xVUAV.bDubinsPathControl = true;
    xVUAV.mTurnrate = turnrate;
    xVUAV.mPitchrate = pitchrate;
    xVUAV.bNormVFVectors=~true;

    %% Create navigational vector fields 
    G = -1;      %Convergence field
    H = 5;      %Circulation field
    L = 0;       %Time-varying field
    
    
    cVFR = VectorField('Spiral',Radius);
    cVFR.G=G;
    cVFR.H=H;
    cVFR.L=L;
    
    cVFR.xc=XOffset;
    cVFR.yc=YOffset;
    cVFR.zc=ZOffset;
    
    cVFR.vel_x=0;
    cVFR.vel_y=0;
    cVFR.vel_z=0;
    
    cVFR.psi = psi;
    cVFR.gam = gam;
    cVFR.lam = lam; 
    cVFR.K = K;
    
    cVFR.bUseVRel = ~true;
    cVFR.bUsePathFunc = ~true;

    %% Create obsticle vector fields
    clear avoidVF;
    avoidVF = {};
    ovfOpt = {};

    [avoidVF, ovfOpt] = makeOVF(ovfXc, ovfYc, 1, ovfTheta,...
        ovfDF{i}, 'Obstacle 1', avoidVF, ovfOpt);

    %% Run simulation
    plot_h_avoid=[];
    RET_VF=[];
    framecount=1;
    
    for k=1:length(t_list)
        t=t_list(k);

        if(bShowVectorField)
            clear opt;
            opt.bCustomRange = 45; 
            opt.bShowCircle=~true;
            opt.bPlotQuiverNorm=true;
            opt.DecayFunc = @NoVDecay;
            opt.CustomNumberOfPoints=15;
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
            uavv.z = uav_v(3);
            cVFR = cVFR.UpdatePosition(t,dt,uavv,opt);
            end
        end
    end
    
%     Com = xVUAV.MX;
%     ComX = Com(1,:);
%     ComY = Com(2,:);
%     ComZ = Com(3,:);
%     ComMag = norm(Com)
%     ComX = Com(1,:)/ComMag;
%     ComY = Com(2,:)/ComMag;
%     ComZ = Com(3,:)/ComMag;
%     
%     hold on
    
%     ind =50;
%     POSITION = xVUAV.mPositionHistory;
%     POSITION(:,1) = [];
%     X = POSITION(1,:);
%     Y = POSITION(2,:);
%     Z = POSITION(3,:);
%     
%     quiver3(X,Y,Z,ComX,ComY,ComZ);
% 
%     for c = 1:ind
%         plot3([X(c),ComX(c)],[Y(c),Y(c)],[Z(c),Z(c)],'r');
% %         plot3([X(c),X(c)],[Y(c),ComY(c)],[Z(c),Z(c)],'b');
% %         plot3([X(c),X(c)],[Y(c),Y(c)],[Z(c),ComZ(c)],'g');
% % %         axis equal
% %     end
%         xlabel('X');
%         ylabel('Y');
%         zlabel('Z');
%         grid on
% %     axis equal
%     zlim([-10,10])

    uavData.name{i} = dfName{i};
    uavData.position{i} = xVUAV.mPositionHistory';
%     uavData.error{i}(:,1) = errX;
%     uavData.error{i}(:,2) = errY;
    
    clear xVUAV.mPositionHistory;
    clear errX;
    clear errY;
end

save(['results\SpiralSim\sim1Vr' num2str(uav_v) '.mat'], 'uavData');

%% Decay functions go here
function G = VInvExp(rrin)
    G = 0.5.^(rrin);
%     if rrin >=1.4
%         G = 0;
%     end
end

function G = VTanh(rrin)
    rrt = 2.*pi.*(1 - rrin);
    G = 0.5.*(tanh(rrt)+1);
%     if rrin >=1.4
%         G = 0;
%     end
end

function G = VLin(rrin)
    G = (-0.5).*rrin + 1;
    G(G < 0) = 0;
%     if rrin >=1.4
%         G = 0;
%     end
end

function G = VGauss(rrin)
    G = 0.5.^((rrin).^2);
%     if rrin >=1.4
%         G = 0;
%     end
end

function G = VInvSq(rrin)
    G = 0.5.*(1./rrin).^2;
%     if rrin >=1.4
%         G = 0;
%     end
end

function G = NoVDecay(rrin)
    G = ones(1,length(rrin));
end