%=========================================================================
% waypointGuidance.m
%=========================================================================

clc
clear
close all
format compact

   Waypoint = true;
   GVF = true;

   uav = UAV();
   uav.plotHeading = false;
   uav.plotCmdHeading = false;
   
   uavXStart = -285;
   uavYStart =  0;
   uavXEnd   = - uavXStart;
   uavYEnd   =   uavYStart;
   uavVvelocity = 40;
   startHeading = 0;
   turnrate = 0.35;
   dt = 0.025;
   t_list=0:dt:20;
   turn_radius = uavVvelocity / turnrate;
   
   uav.plotUAV = false;
   uav.plotUAVPath = true;
   uav.plotFlightEnv = false;
   uav = uav.setup(uavXStart, uavYStart, uavVvelocity, startHeading, dt, turn_radius);
   
   obstR = 150;
   obstX = 0;
   obstY = 0;
   
   optPath = genOptPath(uav,obstR,obstX,obstY,25);
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
       clf
       hold on
       uav.pltUAV();
       wpMan.pltWpts();
       theta = linspace(0,2*pi);
       r = obstR - obstR * 0.2;
       x = r*cos(theta);
       y = r*sin(theta);  
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
%         legend(h,'Mission Start','Obstacle','Line Segments','Waypoints','Mission End','Waypoint UAV Path','GVF UAV Path');
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

    uavXc = uavXStart;
    uavYc = uavYStart;
    uavTheta = startHeading;
    dt = 0.005;
    t_list=0:dt:20;

    %% Obstacle field initial conditions
    ovfXc = obstX;
    ovfYc = obstY;
    ovfR  = obstR - obstR * 0.2;

    ovfTheta = atan2((ovfYc - uavYc),(ovfXc - uavXc));
    ovfDF = {@VTanh};    %Function
    dfName = {'Hyperbolic Tangent'};
    
    uavColor = {[ 0    0.4470    0.7410],...
                [0.8500    0.3250    0.0980],...
                [0.9290    0.6940    0.1250],...
                [0.4940    0.1840    0.5560],...
                [0.4660    0.6740    0.1880]};

    %% Animation options
    bShowVectorField=false;
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
        
        hold on;
        %% Create navigational vector fields
        G = -1;   % Convergence field
        H = -10;   % Circulation field
        L =  0;   % Time-varying field

        cVFR = CircleVectorField('Straight',ovfR);
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

        [avoidVF, ovfOpt] = makeOVF(ovfXc, ovfYc, ovfR, ovfTheta,...
            ovfDF{i}, 'Obstacle 1', avoidVF, ovfOpt)
        
        opt.bCustomRange = 600; 
        opt.bShowCircle=~false;
        opt.bPlotQuiverNorm=false;
        opt.DecayFunc = @NoVDecay;
        opt.CustomNumberOfPoints=50;
        opt.bNormVFVectors = ~false;
        opt.Color = [0 0 1];
        RET_VF = cVFR.PlotFieldAroundRadius(gca,opt);
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
                opt.bShowCircle=~false;
                opt.bPlotQuiverNorm=false;
                opt.DecayFunc = @VTanh;
                opt.CustomNumberOfPoints=50;
                opt.Color = [0 0 1];
                opt.UAV = xVUAV;

                for ii=1:length(avoidVF)
                    ovfOpt{ii}.bPlotQuiverNorm = false;
                    ovfOpt{ii}.bShowCircle=false;
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
                xVUAV = xVUAV.UpdateControlFromVF(cVFR,t,opt,ovfR);

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
        uavData.position{i} = xVUAV.mPositionHistory';
    end

    for ii=1:length(avoidVF)
        ovfOpt{ii}.bPlotQuiverNorm = false;
        ovfOpt{ii}.bShowCircle=false;
        ovfOpt{ii}.bCustomRange = avoidVF{ii}.plotrange;
        ovfOpt{ii}.bCustomCenter = avoidVF{ii}.plotcenter;
        ovfOpt{ii}.bCustomCircleRadiusPlot = avoidVF{ii}.plotradius;
        ovfOpt{ii}.bNormVFVectors = false;
        ovfOpt{ii}.Color = [1 0 0];
        RET_temp = avoidVF{ii}.VF.PlotFieldAroundRadius(gca,ovfOpt{ii});
        plot_h_avoid = RET_temp.H;
    end
    
    h(7) = scatter(uavData.position{i}(:,1),uavData.position{i}(:,2), '.');
    axis equal;
    xlim([-300 300]);
    ylim([-300 300]);
    set(gca, 'FontSize', 18)
    grid on;
    xlabel('X-Position [m]', 'FontSize', 14);
    ylabel('Y-Position [m]', 'FontSize', 14);
    legend(h,'Mission Start','Obstacle','Line Segments','Waypoints','Mission End','Waypoint UAV Path','GVF UAV Path');
       
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


    
    
   
   
   
   
            
            