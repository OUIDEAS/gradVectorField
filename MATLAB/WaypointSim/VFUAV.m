classdef VFUAV
    properties 
        mPositionHistory
        %mVelocity
        mVelocityVHistory
        mHeadingHistory
        mTurnrate
        uav_v_range = [1 8];
        bVFControlHeading=~true;
        bVFControlVelocity=true;
        bDubinsPathControl=true;
        Wind = false;
        WindMag = 0;
        m_dt;
        mID;
        bNormVFVectors=false;
        mLegendName;
        mPlotColor;
    end
    methods
        function obj = VFUAV(dt)
            obj.mPositionHistory=[];
            obj.mVelocityVHistory=[];
            obj.mHeadingHistory=[];
            obj.m_dt = dt;
        end
        function [theta_rad]=GetHeading(obj)
            spot = length(obj.mHeadingHistory);
            if(~isnan(spot) && spot > 0)
                theta_rad = obj.mHeadingHistory(spot);
            else
                theta_rad = NaN;
            end
        end
        function [pos]=GetPosition(obj)
            spot = length(obj.mPositionHistory(1,:));
            if(~isnan(spot) && spot > 0)
                spot = length(obj.mPositionHistory(1,:));
                pos = obj.mPositionHistory(:,spot);
            else
                pos = NaN;
            end
        end
        function [velV]=GetVelocityV(obj)
            spot = length(obj.mVelocityVHistory(1,:));
            if(~isnan(spot) && spot > 0)
                spot = length(obj.mVelocityVHistory(1,:));
                velV = obj.mVelocityVHistory(:,spot);
            else
                velV = NaN;
            end
        end
        function obj=SetPosition(obj,newPos)
            obj.mPositionHistory=[obj.mPositionHistory,newPos];
        end
        function obj=SetHeading(obj,newHeading)
            obj.mHeadingHistory=[obj.mHeadingHistory;newHeading];
        end

        function obj=SetVelocityAndHeading(obj,u)
            newV = [u.vx;u.vy];
            newT = u.heading;
%             uav_vx = newV*cos(newT);
%             uav_vy = newV*sin(newT);
%             newVV = [uav_vx;uav_vy];
            obj = obj.SetHeading(newT);
            obj.mVelocityVHistory = [obj.mVelocityVHistory,newV];
        end
        function vel = GetMaxTurnrate(obj)
            vel = obj.mTurnrate;
        end
        function obj=UpdateControlFromVF(obj,VF_obj,t,opt,obs_radius)
            %current state of the vehicle
            dt = obj.m_dt;

            theta = obj.GetHeading();
            pos = obj.GetPosition();
            uav_x = pos(1)
            uav_y = pos(2);
            
            if obj.Wind == true
                uav_y = gust(uav_x,uav_y,0,obj.WindMag);
            end
            
            uav_v = obj.GetVelocityV()';
            uav_vx = uav_v(1);
            uav_vy = uav_v(2);
            uav_v=sqrt(uav_v(1)^2+uav_v(2)^2);
            %uav_vx = uav_x+uav_vx*dt;
            %uav_vy = uav_y+uav_vy*dt;
            
            %should match getheading
            %theta = atan2(uav_vy,uav_vx);
            s.t=t;
            s.x=uav_x;
            s.y=uav_y;
            s.uav_vx=uav_vx;
            s.uav_vy=uav_vy;
            s.bNormVFVectors=obj.bNormVFVectors;
            VFres = VF_obj.GetVF_at_XY(s);
            vf_angle_check = atan2(VFres.F(2),VFres.F(1));
            U = VFres.F(1);
            V = VFres.F(2);
            if(~isempty(opt.oVFList))
                %s.DecayFunc = opt.DecayFunc;
                Uavoid=ones(1,length(opt.oVFList));
                Vavoid=ones(1,length(opt.oVFList));

                for k=1:length(opt.oVFList)
                    VFx = opt.oVFList{k}.VF;
                    avoid = VFx.GetVF_at_XY(s);
                    r_at_now = sqrt((uav_x-VFx.xc)^2+(uav_y-VFx.yc)^2);
                    P = opt.DecayFunc(r_at_now,obs_radius);
                    Uavoid(k) = avoid.F(1) * P;
                    Vavoid(k) = avoid.F(2) * P;
                end
                U = VFres.F(1) + sum(Uavoid);
                V = VFres.F(2) + sum(Vavoid);
            end
            vf_angle = atan2(V,U);
            
            %%
            %fprintf('%4.3f (x=%4.2f,y=%4.2f) -> VF %4.2f T %4.2f V %4.2f\n',t,uav_x,uav_y,vf_angle,theta,uav_v);   
            % new position/heading/velocity
            if(obj.bVFControlHeading || obj.bVFControlVelocity)
                if(obj.bVFControlHeading)
                    theta=vf_angle;
                end
%                 if(obj.bVFControlVelocity)          
%                     req_speed = norm(VFres.F)*2;
%                     if(req_speed > obj.uav_v_range(1) && req_speed < obj.uav_v_range(2))
%                         uav_v = req_speed;
%                     elseif(req_speed <= obj.uav_v_range(1))
%                         uav_v = obj.uav_v_range(1);
%                     elseif(req_speed >= obj.uav_v_range(2))
%                         uav_v = obj.uav_v_range(2);
%                     else
%                         error('should not be here VFUAV uav_v');
%                     end
%                 end
            end
            if(obj.bDubinsPathControl && ~obj.bVFControlHeading)
                turnrate=obj.GetMaxTurnrate();
                beta = vf_angle;
                theta = atan2(uav_vy,uav_vx); %update AFTER the new vel (x,y)
                %needs to be used, constrains theta to +/- pi

                if abs(theta - beta) < pi
                    if theta - beta < 0
                        theta = theta + turnrate*dt;            
                    else
                        theta = theta - turnrate*dt;
                    end
                else
                    if theta - beta > 0
                        theta = theta + turnrate*dt;
                    else
                        theta = theta - turnrate*dt;
                    end
                end
            end
            if(~obj.bVFControlHeading && ~obj.bVFControlVelocity && ~obj.bDubinsPathControl) 
                error('must have uav control type');
            end
            uav_vx = uav_v*cos(theta);
            uav_vy = uav_v*sin(theta);
            uav_x = uav_x+uav_vx*dt;
            uav_y = uav_y+uav_vy*dt;
            obj = obj.SetPosition([uav_x;uav_y]);
            uo.vx = uav_vx;
            uo.vy = uav_vy;
            uo.heading = theta;
            obj = obj.SetVelocityAndHeading(uo);
        end
        function err = ComputePositionError(obj,cVF)
            pos = obj.GetPosition();
            uav_x = pos(1);
            uav_y = pos(2);
            err.dist_center = sqrt((uav_x-cVF.xc)^2+(uav_y-cVF.yc)^2);
            err.dist_edge =  err.dist_center-cVF.mCircleRadius;
        end
        function H = PlotPathHistory(obj,ax,color)
            H = scatter(ax,obj.mPositionHistory(1,:),obj.mPositionHistory(2,:),...
                'MarkerFaceColor',color,'MarkerEdgeColor',color,...
                'LineWidth',1);
        end
    end
end