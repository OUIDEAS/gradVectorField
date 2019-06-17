%=========================================================================
% waypointGuidance.m
%=========================================================================

clc
clear
close all
format compact

Waypoint = true;
GVF = true;
Wind = false

uav = UAV();
uav.plotHeading = false;
uav.plotCmdHeading = false;
uav.Wind = Wind;
uav.WindMag = 0.3;

uavXStart = -285;
uavYStart =  0;
uavXEnd   = - uavXStart;
uavYEnd   =   uavYStart;
uavVvelocity = 20;
startHeading = 0;
turnrate = 0.40;
dt = 0.05;
t_list=0:dt:40;
turn_radius = uavVvelocity / turnrate;

uav.plotUAV = false;
uav.plotUAVPath = true;
uav.plotFlightEnv = false;
uav = uav.setup(uavXStart, uavYStart, uavVvelocity, startHeading, dt, turn_radius);

obstRadius = 150;
obstX = 0;
obstY = 0;

optPath = genOptPath(uav,obstRadius,obstX,obstY,25);
%    optPath = [0 50; 200 50]

wpMan = wpt();
wpMan = wpMan.setup(optPath);
wpMan.WPx(end+1) = uavXEnd;
wpMan.WPy(end+1) = uavYEnd;
   
%   WAYPOINT SIM

if Waypoint == true

%    while wpMan.currentWP <= length(wpMan.WPx) && wpMan.active
    for k=1:length(t_list)
    t=t_list(k)
       
       wpMan = wpMan.getWPT(uav.x,uav.y);
       heading = atan2(wpMan.wpy-uav.y,wpMan.wpx-uav.x); 
       uav = uav.update_pos(heading);
       uav.colorMarker = 'r.';
       wpMan.currentWP;         
       theta = linspace(0,2*pi);
       r = obstRadius;% * 0.2;
       x = r*cos(theta);
       y = r*sin(theta);   
    end
    
    hold on    
    uav.pltUAV();    
    wpMan.pltWpts();       
    plot(x,y,'--')

    scatter(uavXStart, uavYStart, 200,'d','filled','b')
    scatter(uavXEnd, uavYEnd, 400,'c','p','filled')

    plot([uavXStart,wpMan.WPx(1)],[uavYStart 0],'k')
    plot([wpMan.WPx,wpMan.WPx],[wpMan.WPy,wpMan.WPy],'k')
    plot(wpMan.WPx,wpMan.WPy,'r*');

    h = zeros(6, 1);
    h(1) = scatter(NaN,NaN,'d','MarkerFaceColor','b','MarkerEdgeColor','b');
    h(2) = plot(NaN,NaN,'b','LineStyle','--');
    h(3) = plot(NaN,NaN,'k');
    h(4) = plot(NaN,NaN,'r*');
    h(5) = scatter(NaN,NaN,'p','MarkerFaceColor','c','MarkerEdgeColor','c');
    h(6) = plot(NaN,NaN,'r.');
        
    if GVF ~= true
       legend(h,'Mission Start','Obstacle','Line Segments','Waypoints','Mission End','Waypoint UAV Path');
       legend show
       xlabel('East(m)')
       ylabel('North(m)')
       axis equal
       grid on    
   end
end

if GVF == true
    %% UAV parameters
    uav_v = uavVvelocity;
    turnrate = turnrate * 180/pi;

    %uavXc = uavXStart;
    %uavYc = uavYStart;
    %uavTheta = startHeading;
    dt = 0.005;
    t_list=0:dt:40;

    %% Obstacle field initial conditions
    ovfXc = obstX;
    ovfYc = obstY;
    ovfRadius  = obstRadius;
    ovfDecayRadius = obstRadius * 0.7;

    ovfTheta = atan2((ovfYc - uavYStart),(ovfXc - uavXStart));
    ovfDF = @VTanh;    %Function
    dfName = {'Hyperbolic Tangent'};
    
    uavColor = {[ 0    0.4470    0.7410],...
                [0.8500    0.3250    0.0980],...
                [0.9290    0.6940    0.1250],...
                [0.4940    0.1840    0.5560],...
                [0.4660    0.6740    0.1880]};

    %% Animation options
    bShowVectorField=false;
    bRunUAV=true;
    xVUAV = VFUAV(dt);
    xVUAV = xVUAV.SetPosition([uavXStart ; uavYStart]);
    uavTheta = 0;
    uo.vx = uav_v*cos(uavTheta);
    uo.vy = uav_v*sin(uavTheta);
    uo.heading = uavTheta;
    xVUAV = xVUAV.SetVelocityAndHeading(uo); 
    clear uo;
    xVUAV.bVFControlVelocity=~true;
    xVUAV.bVFControlHeading=~true;
    xVUAV.bDubinsPathControl = true;
    xVUAV.mTurnrate = turnrate;
    xVUAV.bNormVFVectors=~true;
    xVUAV.Wind = uav.Wind;
    xVUAV.WindMag = uav.WindMag;
    %% Create navigational vector fields
    G = 1;   % Convergence field
    H = 1;   % Circulation field
    L =  0;   % Time-varying field
    line_theta = deg2rad(90);
    cVFR = CircleVectorField('Straight',ovfRadius);
    cVFR.G = G;
    cVFR.H = H;
    cVFR.L=L;
    cVFR.xc=0;
    cVFR.yc=0;
    cVFR.vel_x=0;
    cVFR.vel_y=0;
    cVFR.bUseVRel = ~true;
    cVFR.bUsePathFunc = ~true;
    %% Create obstacle vector fields
    clear avoidVF;
    avoidVF = {};
    ovfOpt = {};

    [avoidVF, ovfOpt] = makeOVF(ovfXc, ovfYc, ovfRadius, ovfTheta,...
        ovfDF, 'Obstacle 1', avoidVF, ovfOpt)

    opt.bCustomRange = 600; 
    opt.bShowCircle=~true;
    opt.bPlotQuiverNorm=true;
    opt.DecayFunc = @NoVDecay;
    opt.CustomNumberOfPoints=20;
    opt.bNormVFVectors = ~true;
    opt.Color = [0 0 1];
    UAVList = {xVUAV};
    VFList = {cVFR};
%     figure;hold on;
    for i = 1:length(UAVList)
        opt.line_theta = line_theta;
        RET_VF = VFList{i}.PlotFieldAroundRadius(gca,opt);
        plot_h_vf = RET_VF.H;
        
        %% Run simulation
        plot_h_avoid=[];
        RET_VF=[];
        framecount=1;

        for k=1:length(t_list)
            t=t_list(k);

            if(bShowVectorField)
                clear opt;
                opt.bCustomRange = 600; 
                opt.bShowCircle=~true;
                opt.bPlotQuiverNorm=true;
                opt.DecayFunc = @VTanh;
                opt.CustomNumberOfPoints=50;
                opt.Color = [0 0 1];
                opt.UAV = UAVList{i};
                
                ovfOpt{ii}.bPlotQuiverNorm = true;
                ovfOpt{ii}.bShowCircle=true;
                ovfOpt{ii}.bCustomRange = avoidVF.plotrange;
                ovfOpt{ii}.bCustomCenter = avoidVF.plotcenter;
                ovfOpt{ii}.bCustomCircleRadiusPlot = avoidVF.plotradius;
                ovfOpt{ii}.Color = [1 0 0];
                ovfOpt{ii}.UAV = UAVList{i};  
            end

            if(bRunUAV)
                pos = UAVList{i}.GetPosition();
                errX(k) = pos(1) - (t.*uav_v);
                errY(k) = pos(2) - 0;

                opt.DecayFunc = @VTanh;
                opt.oVFList = avoidVF;
                opt.DecayRadius = ovfDecayRadius*2;
                UAVList{i} = UAVList{i}.UpdateControlFromVF(VFList{i},t,opt);

                if(isempty(VFList{i}.radFunc))
                    VFList{i} = VFList{i}.UpdatePosition(t,dt);
                else
%                 uav_v = xVUAV.GetVelocityV()
%                 uavv.x = uav_v(1);
%                 
%                 uavv.y = uav_v(2);
%                 uavv.y = gust(uavv.x,uavv.y,0,0.5)
                %cVFR = cVFR.UpdatePosition(t,dt,uavv,opt);
                end
            end
        end
        %uavData.position{i} = xVUAV.mPositionHistory';
    end

    for ii=1:length(avoidVF)
        ovfOpt{ii}.bPlotQuiverNorm = true;
        ovfOpt{ii}.bShowCircle=true;
        ovfOpt{ii}.bCustomRange = avoidVF{ii}.plotrange;
        ovfOpt{ii}.bCustomCenter = avoidVF{ii}.plotcenter;
        ovfOpt{ii}.bCustomCircleRadiusPlot = avoidVF{ii}.plotradius;
        ovfOpt{ii}.bNormVFVectors = true;
        ovfOpt{ii}.Color = [1 0 0];
%         ovfOpt{ii}.DecayFunc = @VTanh;
        ovfOpt{ii}.line_theta = line_theta;
        RET_temp = avoidVF{ii}.VF.PlotFieldAroundRadius(gca,ovfOpt{ii});
        plot_h_avoid = RET_temp.H;
    end
    
    h(7) = scatter(UAVList{i}.mPositionHistory(1,:),UAVList{i}.mPositionHistory(2,:), '.');
    axis equal;
    xlim([-300 300]);
    ylim([-300 300]);
    set(gca, 'FontSize', 18)
    grid on;
    xlabel('X-Position [m]', 'FontSize', 14);
    ylabel('Y-Position [m]', 'FontSize', 14);
    
    if Waypoint == true
        legend(h,'Mission Start','Obstacle','Line Segments','Waypoints','Mission End','Waypoint UAV Path','GVF UAV Path');
    else
        legend('Mission Start','Obstacle','Line Segments','Waypoints','Mission End','Waypoint UAV Path','GVF UAV Path');
    end
       
end
 
    %% Decay functions go here
function G = VTanh(rrin,obs_radius)
    r_non = rrin/obs_radius;
    Ra = 1;
    A = 0.5;
    G = A * (tanh(2 * pi * (Ra - r_non)) + 1);
end

function G = NoVDecay(rrin,obs_radius)
    G = ones(1,length(rrin));
end


    
    
   
   
   
   
            
            