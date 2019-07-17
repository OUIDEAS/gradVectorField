%=========================================================================
% Waypoint vs. GVF in wind
%=========================================================================

clc
clear
close all
format compact
figure; hold all

Waypoint = true;
GVF  = true;
Wind = true;
plot_total_field = false;

uav = WPUAV();
uav.plotHeading = false;
uav.plotCmdHeading = false;

uav.Wind = Wind;
uav.WindDisturbance = 0.25;
uav.WindCenterX = 0;
uav.WindCenterY = 0;
uav.WindXRange = 75;
uav.WindYRange = 300;

uavXStart = -350;
uavYStart =  0;
uavXEnd   = - uavXStart;
uavYEnd   =   uavYStart;
uavVvelocity = 20;
startHeading = 0;
turnrate = 0.35;
time = 125;
dt = 0.05;
t_list=0:dt:time;
turn_radius = uavVvelocity / turnrate;

uav.plotUAV       = false;
uav.plotUAVPath   = true;
uav.plotFlightEnv = false;
uav = uav.setup(uavXStart, uavYStart, uavVvelocity, startHeading, dt, turn_radius);

obstRadius = 125;
obstX = 0;
obstY = 0;

optPath = genOptPath(uav,obstRadius,obstX,obstY,25);

wpMan = wpt();
wpMan = wpMan.setup(optPath);
wpMan.WPx(end+1) = uavXEnd;
wpMan.WPy(end+1) = uavYEnd;  

%   WAYPOINT SIM
if Waypoint == true
    for k=1:length(t_list)
        t=t_list(k);    
        wpMan = wpMan.getWPT(uav.x,uav.y);
        heading = atan2(wpMan.wpy-uav.y,wpMan.wpx-uav.x); 
        uav = uav.update_pos(heading, wpMan);
        uav.colorMarker = 'r:';
        wpMan.currentWP;         
        r = obstRadius;
        r_decay = r * 0.7; 
    end 
    
    %PLOT UAV
    uav.pltUAV();    
    wpMan.pltWpts();
    
    % PLOT CIRCLES
    circle(0+obstX,0+obstY,r,'k--');
    circle(0+obstX,0+obstY,r_decay,'k');
    
    % PLOT WAYPOINTS
    plot(wpMan.WPx,wpMan.WPy,'MarkerSize',10,'Marker','*','LineWidth',2,'LineStyle','none',...
    'Color',[0 0 0]);
end

if GVF == true
    %% UAV setup
    bRunUAV = true;
    uav_v = uavVvelocity;
    xVUAV = VFUAV(dt);
    xVUAV = xVUAV.SetPosition([uavXStart ; uavYStart]);
    uavTheta = deg2rad(0);
    uo.vx = uav_v*cos(uavTheta);
    uo.vy = uav_v*sin(uavTheta);
    uo.heading = uavTheta;
    
    xVUAV = xVUAV.SetVelocityAndHeading(uo,uav_v); 
    clear uo;
    xVUAV.bVFControlVelocity=~true;
    xVUAV.bVFControlHeading=~true;
    xVUAV.bDubinsPathControl = true;
    xVUAV.mTurnrate = turnrate;
    xVUAV.bNormVFVectors = ~true;
    
    
    % VF Wind setup
    xVUAV.Wind    = uav.Wind;
    xVUAV.WindDisturbance = uav.WindDisturbance;
    xVUAV.WindCenterX = uav.WindCenterX;
    xVUAV.WindCenterY = uav.WindCenterY;
    xVUAV.WindXRange  = uav.WindXRange;
    xVUAV.WindYRange  = uav.WindYRange;


    %% Obstacle field initial conditions
    ovfXc = obstX;
    ovfYc = obstY;
    ovfRadius  = obstRadius;
    ovfDecayRadius = r_decay;
    ovfTheta = atan2((ovfYc - uavYStart),(ovfXc - uavXStart));
    ovfDF = @VTanh;    %Function
    dfName = {'Hyperbolic Tangent'};
      
    
    %% Create path fields    
    line_theta = deg2rad(90);
    cVFR = GradientVectorField('Straight',1);
    cVFR.G = 1;
    cVFR.H = 30;
    cVFR.L = 0;
    cVFR.xc = 0;
    cVFR.yc = 0;
    cVFR.vel_x = 0;
    cVFR.vel_y = 0;
    cVFR.bUseVRel = ~true;
    cVFR.bUsePathFunc = ~true;
    cVFR.line_theta = line_theta;
    
    
    %% Create obstacle fields
    clear avoidVF;
    avoidVF = {};
    ovfOpt = {};

    avoidVF = GradientVectorField('Gradient',ovfRadius);
    avoidVF.G = -1;
    avoidVF.H  = 30;
    avoidVF.L =  0;
    avoidVF.xc = 0;
    avoidVF.yc = 0;
    
    avoidVF2 = GradientVectorField('Gradient',ovfRadius);
    avoidVF2.G = -1;
    avoidVF2.H  = 30;
    avoidVF2.L =  0;
    avoidVF2.xc = 0;
    avoidVF2.yc = 0;
    
    % Add UAVs to list
    UAVList = {xVUAV};
    
    % Add obstacles and paths to list
    VFList = [cVFR,avoidVF];
    
    for i = 1:length(UAVList)
        for k=1:length(t_list)
            t=t_list(k);
            % Main tasks are happening here
            if(bRunUAV)
                pos = UAVList{i}.GetPosition();
                UAV_X_POS(k) =  pos(1);
                UAV_Y_POS(k) =  pos(2);
                errX(k) = pos(1) - (t.*uav_v);
                errY(k) = pos(2) - 0;
                opt.DecayFunc = @VTanh;
                opt.oVFList = VFList;
                opt.bNormVFVectors = false;
                opt.DecayRadius = ovfDecayRadius;
                UAVList{i} = UAVList{i}.UpdateControlFromVF(VFList(i),t,opt);
            end
        end
    end
    
    % PLOTTING UAV
    scatter(UAV_X_POS, UAV_Y_POS,'b.')
    axis equal;
    xlim([uavXStart -uavXStart]);
    ylim([uavXStart -uavXStart]);
    set(gca, 'FontSize', 18)
    grid on;
    xlabel('X-Position [m]', 'FontSize', 14);
    ylabel('Y-Position [m]', 'FontSize', 14);
  
end

if plot_total_field == true
        % PLOTTING TOTAL FIELD...
        x_list = linspace(uavXStart,-uavXStart,50);
        y_list = linspace(uavXStart,-uavXStart,50);
        s.uav_vx=0;
        s.uav_vy=0;
        s.bNormVFVectors=~true;
        s.line_theta =line_theta;
        k=1;
        Vect=[];
        for i=1:length(x_list)
            for ii=1:length(y_list)
                y=y_list(ii);
                s.x = x_list(i);
                s.y = y_list(ii);
                VFresPlot = cVFR.GetVF_at_XY(s);
                U(i,ii) = VFresPlot.F(1);
                V(i,ii) = VFresPlot.F(2);
                if(~isempty(opt.oVFList))  
                    Uavoid=zeros(1,length(avoidVF));
                    Vavoid=zeros(1,length(avoidVF));
                    for k=1:length(opt.oVFList)
                        VFx = opt.oVFList(k);
                        avoid = VFx.GetVF_at_XY(s);
                        r_at_now = sqrt((s.x-obstX).^2+(s.y-obstY).^2);
                        P = opt.DecayFunc(r_at_now,obstRadius);
                        Uavoid(k) = avoid.F(1) * P;
                        Vavoid(k) = avoid.F(2) * P;
%                         circle(VFx.xc,VFx.yc,ovfRadius);
                    end
                end
                X(i,ii) = s.x;
                Y(i,ii) = s.y;
                % Avoidance plotting
                AvoidU(i,ii) = sum(Uavoid);
                AvoidV(i,ii) = sum(Vavoid);
                    
                % Path plotting
                UPath(i,ii) = VFresPlot.F(1);
                VPath(i,ii) = VFresPlot.F(2);
                
                MagPath(i,ii) = sqrt(UPath(i,ii).^2 + VPath(i,ii).^2);
                NormPathU(i,ii) = UPath(i,ii)/MagPath(i,ii);
                NormPathV(i,ii) = VPath(i,ii)/MagPath(i,ii);
                
                % Total Field Plotting
                U(i,ii) = UPath(i,ii) +  AvoidU(i,ii);
                V(i,ii) = VPath(i,ii) +  AvoidV(i,ii);
                
                magUV(i,ii) = sqrt(U(i,ii).^2 + V(i,ii).^2);
                normU(i,ii) = U(i,ii)/magUV(i,ii);
                normV(i,ii) = V(i,ii)/magUV(i,ii);
            end  
        end
%         h(9) = quiver(X,Y,NormPathU,NormPathV,0.5,'Color','b');
%         h(10) = quiver(X,Y,AvoidU,AvoidV,'Color','r');
        h(11) = quiver(X,Y,normU,normV,'Color','b');
end

%% Decay functions go here
function G = VTanh(rrin,obs_radius)
    r_non = rrin / obs_radius;
    Ra = 1.0;
    A  = 0.5;
    G = A * (tanh(2 * pi * (Ra - r_non)) + 1);
end

function G = NoVDecay(rrin,obs_radius)
    G = ones(1,length(rrin));
end

% CIRCLE PLOTTER
function h = circle(x,y,r,LineType)
    hold on
    th = 0:0.075:2*pi;
    xunit = r * cos(th) + x;
    yunit = r * sin(th) + y;
    h = plot(xunit, yunit, LineType);
end


    
    
   
   
   
   
            
            