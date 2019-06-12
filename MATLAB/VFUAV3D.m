classdef VFUAV3D
    properties 
        mPositionHistory
        %mVelocity
        mVelocityVHistory
        mHeadingHistory
        mPitchHistory
        mTurnrate
        mPitchrate
        MX
        uav_v_range = [-2 2];
        bVFControlHeading=~true;
        bVFControlVelocity=true;
        bDubinsPathControl=true;
        m_dt;
        mID;
        bNormVFVectors=false;
        mLegendName;
        mPlotColor;
    end
    methods
        function obj = VFUAV3D(dt)
            obj.mPositionHistory=[];
            obj.mVelocityVHistory=[];
            obj.mHeadingHistory=[];
            obj.mPitchHistory = [];
            obj.m_dt = dt;
            obj.MX = [];
        end
        function [theta_rad]=GetHeading(obj)
            spot = length(obj.mHeadingHistory);
            if(~isnan(spot) && spot > 0)
                theta_rad = obj.mHeadingHistory(spot);
            else
                theta_rad = NaN;
            end
        end
        
       function [pitch_rad]=GetPitch(obj)
            spot = length(obj.mPitchHistory);
            if(~isnan(spot) && spot > 0)
                pitch_rad = obj.mPitchHistory(spot);
            else
                pitch_rad = NaN;
            end
       end

      %----------------------------------
      function [M_x] = GetXvf(obj)
            spot = length(obj.MX);
            if(~isnan(spot) && spot > 0)
                M_x = obj.MX(spot);
            else
                M_x = NaN;
            end
      end 
      %-----------------------------------
         
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
        
        function obj=SetMX(obj,Xvf)
            commands = [Xvf.X;Xvf.Y;Xvf.Z];
            obj.MX   = [obj.MX,commands];
        end

        function obj=SetPosition(obj,newPos)
            obj.mPositionHistory=[obj.mPositionHistory,newPos];
        end
        function obj=SetHeading(obj,newHeading)
            obj.mHeadingHistory=[obj.mHeadingHistory;newHeading];
        end
        function obj=SetPitch(obj,newPitch)
            obj.mPitchHistory=[obj.mPitchHistory;newPitch];
        end

        function obj=SetVelocityAndHeading(obj,u)
            newV = [u.vx;u.vy;u.vz];
            newT = u.heading;
            newP = u.pitch;
%             uav_vx = newV*cos(newT);
%             uav_vy = newV*sin(newT);
%             newVV = [uav_vx;uav_vy];
            obj = obj.SetHeading(newT);
            obj = obj.SetPitch(newP);
            obj.mVelocityVHistory = [obj.mVelocityVHistory,newV];
        end
        function vel = GetMaxTurnrate(obj)
            vel = obj.mTurnrate;
        end
        function vel_pitch = GetMaxPitchrate(obj)
            vel_pitch = obj.mPitchrate;
        end
            
        function obj=UpdateControlFromVF(obj,VF_obj,t,opt)
            %current state of the vehicle
            dt = obj.m_dt;

            theta = obj.GetHeading();
            pitch = obj.GetPitch();
            
            pos = obj.GetPosition();
            Com = obj.GetXvf();
            
            uav_x = pos(1);
            uav_y = pos(2);
            uav_z = pos(3);
            
            uav_v = obj.GetVelocityV()';
            uav_vx = uav_v(1);
            uav_vy = uav_v(2);
            uav_vz = uav_v(3);
                       
            uav_v=sqrt(uav_v(1)^2+uav_v(2)^2+uav_v(3)^2);
            
            %uav_vx = uav_x+uav_vx*dt;
            %uav_vy = uav_y+uav_vy*dt;
            
            %should match getheading
            %theta = atan2(uav_vy,uav_vx);
            s.t=t;
            
            s.x=uav_x;
            s.y=uav_y;
            s.z=uav_z;
            
            s.uav_vx=uav_vx;
            s.uav_vy=uav_vy;
            s.uav_vz=uav_vz;
            
            s.bNormVFVectors=obj.bNormVFVectors;
            VFres = VF_obj.GetVF_at_XY(s);
%             vf_angle_check = atan2(VFres.F(2),VFres.F(1));
            
            U = VFres.F(1);
            V = VFres.F(2);
            W = VFres.F(3);
            
            if(~isempty(opt.oVFList))
                %s.DecayFunc = opt.DecayFunc;
                Uavoid=ones(1,length(opt.oVFList));
                Vavoid=ones(1,length(opt.oVFList));
                Wavoid=ones(1,length(opt.oVFList));

                for k=1:length(opt.oVFList)
                    VFx = opt.oVFList{k}.VF;
                    avoid = VFx.GetVF_at_XY(s);
                    r_at_now = sqrt((uav_x-VFx.xc)^2+(uav_y-VFx.yc)^2);
                    P = opt.DecayFunc(r_at_now);
                    Uavoid(k) = avoid.F(1);
                    Vavoid(k) = avoid.F(2);
                    Wavoid(k) = avoid.F(3);
                end
                U = VFres.F(1) + sum(Uavoid);
                V = VFres.F(2) + sum(Vavoid);
                W = VFres.F(3) + sum(Wavoid);
            end
%             W = 0;
            vf_angle   = atan2(V,U);          
            vf_pitch   = atan2(sqrt(U.^2+V.^2),W)-pi/2;

%             pitch_tan = atan2(
                       
            
            %%
            %fprintf('%4.3f (x=%4.2f,y=%4.2f) -> VF %4.2f T %4.2f V %4.2f\n',t,uav_x,uav_y,vf_angle,theta,uav_v);
            
            %update new position/heading/velocity
            if(obj.bVFControlHeading || obj.bVFControlVelocity)
                if(obj.bVFControlHeading)
                    theta=vf_angle;
                    pitch=vf_pitch;
                end

            end
            if(obj.bDubinsPathControl && ~obj.bVFControlHeading)
                turnrate=obj.GetMaxTurnrate();
                pitchrate= obj.GetMaxPitchrate();
                
                beta  = vf_angle;
                alpha = vf_pitch;
                
                theta = atan2(uav_vy,uav_vx); %update AFTER the new vel (x,y)
                pitch = atan2(sqrt(uav_vy.^2+uav_vx.^2),uav_vz)-pi/2;
                

               
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
                
                if abs(pitch - alpha) < pi
                    if pitch - alpha < 0
                        pitch = pitch + pitchrate*dt;    
                    else
                        pitch = pitch - pitchrate*dt;
                    end
                else
                    if pitch - alpha > -pi
                        pitch = pitch + pitchrate*dt;
                    else
                        pitch = pitch - pitchrate*dt;
                    end
                end
 
            end
            if(~obj.bVFControlHeading && ~obj.bVFControlVelocity && ~obj.bDubinsPathControl) 
                error('must have uav control type');
            end
            uav_vx = uav_v*cos(theta)*cos(pitch);
            uav_vy = uav_v*sin(theta)*cos(pitch);
            uav_vz = -uav_v*sin(pitch);
%             uav_vz =0;
            
            uav_x = uav_x+uav_vx*dt;
            uav_y = uav_y+uav_vy*dt;
            uav_z = uav_z+uav_vz*dt;
            
            obj = obj.SetPosition([uav_x;uav_y;uav_z]);
           
            
            uo.vx = uav_vx;
            uo.vy = uav_vy;
            uo.vz = uav_vz;
            uo.heading = theta;
            uo.pitch   = pitch;
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