%Normalization of the 3 vectors = tighter control + chatter
%Non-normal = smoother path + more error
%TV + rel velocity creates extreme chattering

close all;
clear variables;
bShowVectorField=true;
bRunUAV=~true;
%bPlotQuiverNorm=~true;
bAnimate=~true;

% xl = -2:0.1:2;
% yl = xl;
% for i = 1:length(xl)
%     for ii = 1:length(yl)
%         x = xl(i);
%         y = yl(ii);
%         r = sqrt(x^2+y^2);
%         z(i,ii) = VDecay(r);
%     end
% end
% [X,Y] = meshgrid(xl);
% surf(X,Y,z);
% set(gca,'ZLim',[-1 1]);
% return;
% r = 0:0.01:1;
% plot(r,VDecay(r));
% VDecay(0)
% VDecay(0.5)
% return;
% r = 0:0.01:1;
% plot(r,VDecayTanh(r));
% return;

r=1;

z=0;
G=1;%conv
H=1;%circ
L=1;%TV
%limit = r*1.5;
%step = 0.5;
%x_list = linspace(-limit,limit,50);%-limit:step:limit;
%y_list = linspace(-limit,limit,50);%-limit:step:limit;

turnrate=[];
uav_v = [];
uav_v_range = [1.5 4];
velx_t = 1;
vely_t = 0;
uav_v = 5*velx_t;
%turnrate = pi;
dt = 1/(uav_v*30);

if(isempty(turnrate))
    turnrate = sqrt((uav_v+velx_t)^2/r^2);
end
if(isempty(uav_v))
    uav_v = sqrt(turnrate^2*r^2)-velx_t;
end

turnrate = turnrate*2;

fig1=figure;    hold on;

%dt = 0.1;
%target initial conditions
x=0;
y=0;
xc=0;
yc=0;
%vel_t=0.01;
%t=0;
theta = 0;
xVUAV = VFUAV(dt);
xVUAV = xVUAV.SetPosition([r*-1.5 ; 0]);
uo.vx = uav_v*cos(theta);
uo.vy = uav_v*sin(theta);
uo.heading = theta;
xVUAV = xVUAV.SetVelocityAndHeading(uo);
clear uo;

dbUAV_Norm = xVUAV;
dbUAV_Norm.bDubinsPathControl=true;
dbUAV_Norm.bVFControlVelocity=~true;
dbUAV_Norm.bVFControlHeading=~true;
dbUAV_Norm.mTurnrate = turnrate;
dbUAV_Norm.bNormVFVectors=true;
dbUAV_Norm.mLegendName = 'Normalizing';
% dbUAV_Norm.mPlotColor = [1 0 0];

dbUAV = xVUAV;
dbUAV.bVFControlVelocity=~true;
dbUAV.bVFControlHeading=~true;
dbUAV.bDubinsPathControl=true;
dbUAV.mTurnrate = turnrate;
dbUAV.bNormVFVectors=false;
dbUAV.mLegendName = 'Lyapunov';

dbUAVnoNorm = xVUAV;
dbUAVnoNorm.bVFControlVelocity=~true;
dbUAVnoNorm.bVFControlHeading=~true;
dbUAVnoNorm.bDubinsPathControl=true;
dbUAVnoNorm.mTurnrate = turnrate;
dbUAVnoNorm.bNormVFVectors=~true;
dbUAVnoNorm.mLegendName = 'Summing';
% dbUAVnoNorm.mPlotColor = [0 0 1];

% UAV_list = {dbUAV_Norm,dbUAV,dbUAVrad};
UAV_list={};
% UAV_list = {UAV_list{:},dbUAV_Norm};
UAV_list = {UAV_list{:},dbUAVnoNorm};
% UAV_list = {UAV_list{:},dbUAV};

%UAV_list = {dbUAV};

cVF = CircleVectorField('Gradient',r);
cVF.G=G;
cVF.H=H;
cVF.L=L;
cVF.xc=0;
cVF.yc=0;
cVF.vel_x=velx_t;
cVF.vel_y=vely_t;
cVF.velPathFunc = @vPath;
cVF.bUsePathFunc = true;
cVF.bUseVRel = ~true;
cVF.mLegendName = 'Summing';

lVF = CircleVectorField('Lyapunov',r);
lVF.G=G;
lVF.H=H;
lVF.L=L;
lVF.xc=0;
lVF.yc=0;
lVF.vel_x=velx_t;
lVF.vel_y=vely_t;
lVF.velPathFunc = @vPath;
lVF.bUsePathFunc = true;
lVF.mLegendName = 'Lyapunov';

cVFR = CircleVectorField('Gradient', r);
cVFR.G=G;
cVFR.H=H;
cVFR.L=L;
cVFR.xc=0;
cVFR.yc=0;
cVFR.vel_x=velx_t;
cVFR.vel_y=vely_t;
cVFR.velPathFunc = @vPath;
cVFR.bUseVRel = ~true;
cVFR.mLegendName = 'Normalizing';
%cVFR.radFunc = @RadFunc;
cVFR.bUsePathFunc = true;
VF_List={};
% VF_List={VF_List{:},cVF};
VF_List={VF_List{:},cVFR};
% VF_List={VF_List{:},lVF};

% VF_List={cVF,lVF,cVFR};

oVFa = CircleVectorField('Gradient',0.01);
oVFa.G=-G;
oVFa.H=0;
oVFa.L=0;
oVFa.xc=0;
oVFa.yc=1.5;
oVFa.vel_x=velx_t;
oVFa.vel_y=vely_t;
oVFa.velPathFunc = @vPath;
oVFa.bUsePathFunc = true;
oVFa.bUseVRel = ~true;
oVFa.mLegendName = 'Obstacle 1';

oVF1.VF = oVFa;
oVF1.plotcenter = [0,1.5];
oVF1.plotrange = 1.5;
oVF1.plotradius = 0.5;

oVFb = CircleVectorField('Gradient',0.01);
oVFb.G=-G*2;
oVFb.H=0;
oVFb.L=0;
oVFb.xc=3.5;
oVFb.yc=1.5;
oVFb.vel_x=velx_t;
oVFb.vel_y=vely_t;
oVFb.velPathFunc = @vPath;
oVFb.bUsePathFunc = true;
oVFb.bUseVRel = ~true;
oVFb.mLegendName = 'Obstacle 2';

oVF2.VF = oVFb;
oVF2.plotcenter = [oVFb.xc,oVFb.yc];
oVF2.plotrange = 1.5;
oVF2.plotradius = 0.5;
avoidVF = {};
avoidVF = {avoidVF{:},oVF1};
avoidVF = {avoidVF{:},oVF2};

% s.x = 0.1;
% s.y = 0;
% s.z = 0;
% s.t=0;
% s.uav_vy = 0;
% s.uav_vx = 1;
% s.bNormVFVectors=false;
% cVF.GetVF_at_XY(s).F
% return
%legend_names = {'Summing','Normalizing','UAV-Summing','UAV-Normalizing','Target'};
legend_names = {'Guidance Field','Obstacle Field','Target','UAV'};
% legend_names = {'Summing','Normalizing','Lyapunov','UAV-Summing','UAV-Normalizing','UAV-Lyapunov','Target'};

color{1} = [0 0 1];
color{2} = [1 0 0];
color{3} = [0.1 1 0.1];
t_list=0:dt:5.5;
%fig1.PaperUnits='inches';
%fig1.PaperPosition = [0 0 12 8];
plot_h_avoid=[];
RET_VF=[];
fig1.Position = [0 0 1200 800];
framecount=1;                               %Used in case the data plotted number is less than all data
for k=1:length(t_list)
    cla;
    t=t_list(k);
    if(bShowVectorField)
        clear opt;
        opt.bShowCircle=~true;
        opt.bPlotQuiverNorm=true;
        opt.DecayFunc = @NoVDecay;
        opt.CustomNumberOfPoints=15;
        opt.bCustomRange = 1.5;
        for ii=1:length(VF_List)
            opt.Color = color{ii};
            opt.UAV = UAV_list{ii};
            RET_VF{ii} = VF_List{ii}.PlotFieldAroundRadius(gca,opt);
            plot_h_vf(ii) = RET_VF{ii}.H;
        end
    end
    
    if(bRunUAV)
        for ii=1:length(UAV_list)
            err{k,ii} = UAV_list{ii}.ComputePositionError(VF_List{ii});
            opt.DecayFunc = @VDecayTanh;
            opt.oVFList = avoidVF;
            
            UAV_list{ii} = UAV_list{ii}.UpdateControlFromVF(VF_List{ii},t,opt);
            if(isempty(VF_List{ii}.radFunc))
                VF_List{ii} = VF_List{ii}.UpdatePosition(t,dt);
            else
                uav_v = UAV_list{ii}.GetVelocityV();
                uavv.x = uav_v(1);
                uavv.y = uav_v(2);
                
                VF_List{ii} = VF_List{ii}.UpdatePosition(t,dt,uavv,opt);
            end
            %hold off;
            plot_vf_center = scatter(VF_List{ii}.xc_history,VF_List{ii}.yc_history,'MarkerFaceColor',[0 0 0],'MarkerEdgeColor',[0 0 0]);   
            %hold on;
        end
          
        if(bAnimate)
            for ii=1:length(UAV_list)
                xUAV = UAV_list{ii};
                plot_h_uav(ii) = xUAV.PlotPathHistory(gca,color{ii});
            end
        end
    end
    axis equal;
    grid on;
    xlabel('X-Position [-]');
    ylabel('Y-Position [-]');
    if(bAnimate)
        
        plot_handles=[];
        if(~isempty(plot_h_vf))
            plot_handles = [plot_handles,plot_h_vf];
        end
        if(~isempty(plot_h_avoid))
            plot_handles = [plot_handles,plot_h_avoid];
        end
        if(~isempty(plot_h_uav))
            plot_handles = [plot_handles,plot_h_uav];
        end
        if(~isempty(plot_vf_center))
            plot_handles = [plot_handles,plot_vf_center];
        end
        legend(plot_handles,legend_names);
        %legend([plot_h_vf,plot_h_uav,plot_vf_center],legend_names);
        %legend([plot_vf_center,plot_h_uav],);
        drawnow();
        F(framecount)=getframe(fig1);           %Save the frame
        framecount=framecount+1;
    end
end

% plot(t_list,xerr(1,:));
% plot(t_list,xerr(2,:));

function vel = vPath(t)
%     vel=[0;0];
%     return;
    v = 1;
    if(t<=1)
        theta = [0,1];
        ang = atan2(theta(2),theta(1));
        vel = v.*[sin(ang);cos(ang)];
    elseif(t<2)
        theta = [1,0];
        ang = atan2(theta(2),theta(1));
        vel = v.*[sin(ang);cos(ang)];
    elseif(t<3)
        theta = [1,1];
        ang = atan2(theta(2),theta(1));
        vel = v.*[sin(ang);cos(ang)];
    elseif(t<4)
        theta = [0,1];
        ang = atan2(theta(2),theta(1));
        vel = v.*[sin(ang);cos(ang)];
    else
        vel = [0;0];
    end

end
function rad = RadFunc(vel_ratio)
    rad = vel_ratio/5;
end

function G = NoVDecay(r)
    G = ones(1,length(r));
end
function G = VDecayLinear(rrin)
    ovfM = 0.5;
    ovfR = 0.5;
    G = ((ovfM-1)./ovfR).*rrin + 1;
    G(G < 0) = 0;
end
function G = VDecayTanh(rrin)
    rrt = -((rrin)*2*pi-pi);
%     G = tanh(rrt)/2+0.5;
    G = tanh(rrt)/2+0.5;
end
