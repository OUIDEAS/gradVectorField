%=========================================================================
% waypointGuidance.m
%
%
%=========================================================================
clc
clear
close all
   uav = UAV();
   uav.plotHeading = false;
   uav.plotCmdHeading = false;
   
   uavXStart = -500;
   uavYStart = -200;
   uavXEnd   = - uavXStart;
   uavYEnd   =   uavYStart;
   uavVvelocity = 35;
   startHeading = 0;
   turnrate = 0.35;
   dt = .5;
   turn_radius = uavVvelocity/turnrate
   
   uav.plotUAV = false;
   uav.plotUAVPath = true;
   uav.plotFlightEnv = false;
   uav = uav.setup(uavXStart, uavYStart, uavVvelocity, startHeading, dt, turn_radius);
   
   obstR = 200;
   obstX = 0;
   obstY = 0;
   
   optPath = genOptPath(uav,obstR,obstX,obstY,25);
%    optPath = [0 50; 200 50]
 
   wpMan = wpt();
   wpMan = wpMan.setup(optPath);
   wpMan.WPx(end+1) = uavXEnd;
   wpMan.WPy(end+1) = uavYEnd;
   
%   WAYPOINT SIM
   while wpMan.currentWP <= length(wpMan.WPx) && wpMan.active
       wpMan = wpMan.getWPT(uav.x,uav.y);
       heading = atan2(wpMan.wpy-uav.y,wpMan.wpx-uav.x);
       uav = uav.update_pos(heading);
       wpMan.currentWP;         
       clf
       hold on
       uav.pltUAV();
       wpMan.pltWpts();
       theta = linspace(0,2*pi);
       r = obstR - obstR * 0.2;
       x = r*cos(theta);
       y = r*sin(theta);  
%        plot(x,y,'--')
       
       scatter(uavXStart,uavYStart,200,'d','filled','b')
       scatter(uavXEnd,uavYEnd,400,'c','p','filled')
       
       plot([uavXStart,wpMan.WPx(1)],[uavYStart 0],'k')
       plot([wpMan.WPx,wpMan.WPx],[wpMan.WPy,wpMan.WPy],'k')
       plot(wpMan.WPx,wpMan.WPy,'r*');
       
        h = zeros(6, 1);
        h(1) = scatter(NaN,NaN,'d','MarkerFaceColor','b','MarkerEdgeColor','b');
        h(2) = plot(NaN,NaN,'b','LineStyle','--');
        h(3) = plot(NaN,NaN,'k');
        h(4) = plot(NaN,NaN,'r*');
        h(5) = scatter(NaN,NaN,'p','MarkerFaceColor','c','MarkerEdgeColor','c');
        h(6) = plot(NaN,NaN,'k.');
        legend(h,'Mission Start','Obstacle','Line Segments','Waypoints','Mission End','UAV Path');
       
       legend show
       xlabel('East(m)')
       ylabel('North(m)')
       axis equal
  
   end
   
   
   
   grid on
   
            
            