classdef CircleVectorField
    properties
        G;
        H;
        L;
        bUseVRel=~true;
        
        vel_x=0;
        vel_y=0;
        
        mCircleRadius=1;
        
        xc;
        yc;
        
        xc_history=[];
        yc_history=[];
        bUsePathFunc=false;
        velPathFunc=[];
        bGradientVF = false;
        bLyapunovVF = false;
        bStraightVF = false;
        radFunc=[];
        mLegendName;
    end
    
    methods
        function obj = CircleVectorField(type,radius_in)
            if(strcmp(type,'Gradient'))
                obj.bGradientVF=true;
            elseif(strcmp(type,'Lyapunov'))
                obj.bLyapunovVF=true;
            elseif(strcmp(type,'Straight'))
                obj.bStraightVF=true;
            else
                error('not a known type')
            end
            obj.mCircleRadius=radius_in;
        end
        function V = GetVF_at_XY(obj,si)
            s.x = si.x;
            s.y = si.y;
            s.z = 0;
            s.uav_vy = si.uav_vy;
            s.uav_vx = si.uav_vx;
            s.G=obj.G;
            s.H=obj.H;
            s.L=obj.L;
            s.bNormVFVectors=si.bNormVFVectors;
            
            s.bUseVRel=obj.bUseVRel;
            s.xc=obj.xc;
            s.yc=obj.yc;
            if(~obj.bUsePathFunc || isempty(obj.velPathFunc))
                s.velx=obj.vel_x;
                s.vely=obj.vel_y;
            else
                vel_v = obj.velPathFunc(si.t);
                s.velx=vel_v(1);
                s.vely=vel_v(2);
            end
            s.r=obj.mCircleRadius;
            if(obj.bGradientVF)
                V = obj.VFtv(s);
            elseif(obj.bLyapunovVF)
                V.F = obj.VFLyapunov(s);
            elseif(obj.bStraightVF)
                V = obj.VFStraight(s);
            else
                error('no VF type');
            end
        end
        function obj = UpdatePosition(obj,t,dt,uavv)
            if(~obj.bUsePathFunc || isempty(obj.velPathFunc))
%                 s.velx=obj.vel_x;
%                 s.vely=obj.vel_y;
            else
                vel_v = obj.velPathFunc(t);
                obj.vel_x=vel_v(1);
                obj.vel_y=vel_v(2);
            end
            obj.xc = obj.xc + obj.vel_x*dt;
            obj.yc = obj.yc + obj.vel_y*dt;
            obj.xc_history=[obj.xc_history;obj.xc];
            obj.yc_history=[obj.yc_history;obj.yc];
            
            if(~isempty(obj.radFunc))
                vel_r = sqrt(uavv.x^2+uavv.y^2) / sqrt(obj.vel_x^2+obj.vel_y^2);
                if(vel_r == inf)
                    vel_r =  1;
                end
                new_r = obj.radFunc(vel_r);
                
                obj.mCircleRadius = new_r;
                fprintf('R->%4.2f\n',new_r);
            end
        end
        function obj = SetPosition(obj,newxy)
            obj.xc = newxy(1);
            obj.yc = newxy(2);
        end
        function RET = PlotFieldAroundRadius(obj,axis,opt)
            limit = obj.mCircleRadius*1.5;
            NumPoints = 0.5;
            if(isfield(opt,'bCustomRange'))
                limit = opt.bCustomRange;
            end
            if(isfield(opt,'CustomNumberOfPoints'))
                NumPoints = opt.CustomNumberOfPoints;
            end
            if(isfield(opt,'bCustomCenter'))
                xct = opt.bCustomCenter(1);
                yct = opt.bCustomCenter(2);
            else
                xct = obj.xc;
                yct = obj.yc;
            end
            VFUAV = [];
            if(isfield(opt,'UAV'))
                VFUAV = opt.UAV;
            end
            x_list = linspace(-limit,limit,NumPoints)+xct;%-limit:step:limit;
            y_list = linspace(-limit,limit,NumPoints)+yct;
            k=1;
            Vect=[];
            for i=1:length(x_list)
                x = x_list(i);
                for ii=1:length(y_list)
                    y=y_list(ii);
                    %alpha1 = x^2+y^2-r^2;
                    %alpha2 = z;
                    %Vconv{i,ii} = -G.*(alpha1(x,y,r).*[2.*x;2.*y;0]+alpha2(z).*[0;0;1]);
                    %Vcirc{i,ii} = H.*[2.*y;-2.*x;0]; %page 81
                    %VF_list{i,ii}=Vconv{i,ii}+Vcirc{i,ii};
                    %  Vconv(x,y,z,xc,yc,r)
                    %  Vcirc(x,y,xc,yc)
                    %  Minv_a(x,y,xc,yc,vel,t)'
                    %fprintf('(x,y) %4.2f,%4.2f\t (xc,yc) %4.2f,%4.2f\t (v,t) %4.2f,%4.2f\n'...
                    %    ,x,y,xc,yc,vel,t);
                    %VF_res = VF(x,y,0,r,G,H);
                    s.x=x;
                    s.y=y;
                    s.z=0;
                    s.xc=obj.xc;
                    s.yc=obj.yc;
                    s.velx=obj.vel_x;
                    s.vely=obj.vel_y;
                    s.r=obj.mCircleRadius;
                    r_at_now = sqrt((x-obj.xc)^2+(y-obj.yc)^2);
                    Rxx(i,ii) = r_at_now;
                    P(i,ii)=1;
                    if(isfield(opt,'DecayFunc'))
                        P(i,ii) = opt.DecayFunc(r_at_now);
                    end
                    s.G=obj.G;
                    s.H=obj.H;
                    s.L=obj.L;
                    if(~isempty(VFUAV))
                        s.bNormVFVectors=VFUAV.bNormVFVectors;
                    %s.Norm=false;
                    %s.bUseVRel=false;
                        uav_vel = VFUAV.GetVelocityV();
                        s.uav_vx=uav_vel(1);
                        s.uav_vy=uav_vel(2);
                    else
                        s.bNormVFVectors=opt.bNormVFVectors;
                        s.uav_vx=0;
                        s.uav_vy=0;
                    end
                    if(obj.bGradientVF)
                        VF_res = obj.VFtv(s);%VFtv(x,y,0,xc,yc,vel_t,0,r,G,H);
                        VF_list_tv{i,ii}=VF_res.tv;
                        VF_list_circ{i,ii}=VF_res.circ;
                        VF_list_conv{i,ii}=VF_res.conv;
                        VF_list{i,ii}=VF_res.F;%VF_res.F;
                    elseif(obj.bLyapunovVF)
                        VF_res = obj.VFLyapunov(s);
                        VF_list{i,ii} = VF_res;
                    elseif(obj.bStraightVF)
                        VF_res = obj.VFStraight(s);
                        VF_list_tv{i,ii}=VF_res.tv;
                        VF_list_circ{i,ii}=VF_res.circ;
                        VF_list_conv{i,ii}=VF_res.conv;
                        VF_list{i,ii} = VF_res.F;
                    end
                    u(i,ii)=VF_list{i,ii}(1);
                    v(i,ii)=VF_list{i,ii}(2);
                    x_q(i,ii)=x;
                    y_q(i,ii)=y;
                    %Vect = [Vect;VF_list{i,ii}(1) VF_list{i,ii}(2)];
                    %k=k+1;
                end
            end
            if(obj.bGradientVF)
                for i=1:length(x_list)
                    for ii=1:length(y_list)
                        ucirc(i,ii)=VF_list_circ{i,ii}(1);
                        vcirc(i,ii)=VF_list_circ{i,ii}(2);

                        uconv(i,ii)=VF_list_conv{i,ii}(1);
                        vconv(i,ii)=VF_list_conv{i,ii}(2);

                        utv(i,ii)=VF_list_tv{i,ii}(1);
                        vtv(i,ii)=VF_list_tv{i,ii}(2);
                    end
                end
            end
            
            %bPlotQuiverNorm=false;
            if(opt.bPlotQuiverNorm)
                normnorm = sqrt(u.^2+v.^2);
                u=u./sqrt(u.^2+v.^2);
                v=v./sqrt(u.^2+v.^2);
%                 
%                 u=u./normnorm;
%                 v=v./normnorm;
                %VectNorm = Vect./norm(Vect,2);
                if(obj.bGradientVF)
%                     circ_norm = sqrt(ucirc.^2+vcirc.^2);
                    ucirc=ucirc./sqrt(ucirc.^2+vcirc.^2);
                    vcirc=vcirc./sqrt(ucirc.^2+vcirc.^2);
                    
%                     conv_norm = sqrt(uconv.^2+vconv.^2);
                    uconv=uconv./sqrt(uconv.^2+vconv.^2);
                    vconv=vconv./sqrt(uconv.^2+vconv.^2);
                    
%                     utv_norm = sqrt(utv.^2+vtv.^2);
                    utv=utv./sqrt(utv.^2+vtv.^2);
                    vtv=vtv./sqrt(utv.^2+vtv.^2);
                end
%              else
%                 un=u;
%                 vn=v;
            end
            
            %scale function for avoid
            for i=1:length(x_list)
                x = x_list(i);
                for ii=1:length(y_list)
                    y=y_list(ii);
                    nu(i,ii) = u(i,ii) .* P(i,ii);
                    nv(i,ii) = v(i,ii) .* P(i,ii);
                    Q(i,ii) = sqrt(u(i,ii)^2+v(ii,ii)^2);
                    QP(i,ii) = sqrt(nu(i,ii)^2+nv(i,ii)^2);
                end
            end
            %only for plotting...
            if(isfield(opt,'sOnlyShow'))
                if(strcmp(opt.sOnlyShow, 'Circ'))
                    nu = ucirc;
                    nv = vcirc;
                elseif(strcmp(opt.sOnlyShow, 'Conv'))
                    nu = uconv;
                    nv = vconv;
                elseif(strcmp(opt.sOnlyShow, 'TV'))
                    nu = utv;
                    nv = vtv;
                end
            end
            
            RET_H = quiver(axis,x_q,y_q,nu,nv,'Color',opt.Color);
            
            if(~isempty(obj.mLegendName))
                set(RET_H,'DisplayName',obj.mLegendName);
            end
            %alpha(1);

    %         quiver(x_q,y_q,ucirc,vcirc,'Color',[1 0 0]);
    %         quiver(x_q,y_q,uconv,vconv,'Color',[0 1 0]);
    %         quiver(x_q,y_q,utv,vtv,'Color',[0 0 0]);
            if(isfield(opt,'bShowCircle') && opt.bShowCircle)
                th = 0:pi/50:2*pi;
                x_c=obj.xc;
                y_c=obj.yc;
                Rc = obj.mCircleRadius;
                if(isfield(opt,'bCustomCircleRadiusPlot'))
                    Rc =  opt.bCustomCircleRadiusPlot;
                end
                xunit = Rc * cos(th) + x_c;
                yunit = Rc * sin(th) + y_c;
                h = plot(axis,xunit, yunit,'HandleVisibility','on','Color','k','LineWidth',2);
            end
            RET.H = RET_H;
            RET.QP = QP;
            RET.Q = Q;
            RET.X = x_q;
            RET.Y = y_q;
            RET.P = P;
            RET.R = Rxx;
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function V = VFStraight(obj,s)
            Vcnv=s.G.*[0;s.y;s.z];
            Vcrc=s.H.*[-1;0;0];
            TV = 0;
            if(s.bNormVFVectors)
                Vcnv = Vcnv/norm(Vcnv);
                if(norm(Vcrc) ~=0)
                    Vcrc = Vcrc/norm(Vcrc);
                end
            end
            V.conv = Vcnv;
            V.circ = Vcrc;
            V.tv = TV;
            V.F=Vcnv+Vcrc+TV;
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function V = VFLyapunov(obj,s) 
            x=s.x-s.xc;
            y=s.y-s.yc;
            rd=s.r;
            r = sqrt(x^2+y^2);
            u = -x*(r^2-rd^2)/(r^2+rd^2) - y*(2*r*rd)/(r^2+rd^2);
            v = -y*(r^2-rd^2)/(r^2+rd^2) + x*(2*r*rd)/(r^2+rd^2);
            V=[u;v];
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function alp1 = alpha1_circ(o,s) 
            alp1 = (s.x-s.xc)^2+(s.y-s.yc)^2-s.r^2;
        end
        function alp2 = alpha2_circ(o,s)
            alp2 = s.z;
        end
        function V = Vconv_c(o,s) 
            V1 = -o.alpha1_circ(s).*[2.*(s.x-s.xc);2.*(s.y-s.yc);0]./s.r;%^2;
            V2 = o.alpha2_circ(s).*[0;0;1];
            V = V1+V2;
%             V = sqrt(V1.^2+V2.^2);
        end
        function V = Vcirc_c(o,s) 
            V = [2.*(s.y-s.yc);-2.*(s.x-s.xc);0];
        end
        function V=VFtv(o,s)
            Vcnv=s.G.*o.Vconv_c(s);
            Vcrc=s.H.*o.Vcirc_c(s);
            TV=-s.L.*(o.Minv_a(s));
            if(s.bNormVFVectors)
                Vcnv = Vcnv/norm(Vcnv);
                if(norm(Vcrc) ~=0)
                    Vcrc = Vcrc/norm(Vcrc);
                end
                if(norm(TV) ~= 0)
                    TV = TV/norm(TV);
                end
            end
            V.conv = Vcnv;
            V.circ = Vcrc;
            V.tv = TV;
            V.F=Vcnv+Vcrc+TV;
        end
        function a = Ma_(o,x,xc)
            a = 2*(x-xc);
        end
        function b = Mb_(o,y,yc)
            b = 2*(y-yc);
        end
        function p = Mp_(o,s) %dAlphadt
            if(o.bUseVRel)
                p=2*(s.x - s.xc).*-(s.uav_vx-s.velx)+2*(s.y - s.yc).*-(s.uav_vy-s.vely);

                %p=-2*(s.uav_vx-s.velx)*(s.x - s.xc)-2*(s.uav_vy-s.vely)*(s.y - s.yc);
            else
                p=  -2*s.velx*(s.x - s.xc)-2*s.vely*(s.y - s.yc);

                %p=(-2*s.velx*(s.x - s.xc)-2*s.vely*(s.y - s.yc))/s.r^2;
            end
        end
        function Mia = Minv_a(o,s)
            %Mia = o.Mp_(s);
            %dAlpha1 = [o.Ma_(s.x,s.xc);0;o.Mb_(s.y,s.yc)];
            %Mia = Mia * dAlpha1/norm(dAlpha1).^2;
            
            Mia = o.Mp_(s)/(o.Ma_(s.x,s.xc)^2+o.Mb_(s.y,s.yc)^2)*[o.Ma_(s.x,s.xc); o.Mb_(s.y,s.yc);0];

            %Mia = o.Mp_(s)/(o.Ma_(s.x,s.xc)^2+o.Mb_(s.y,s.yc)^2)*[o.Ma_(s.x,s.xc); o.Mb_(s.y,s.yc);0];
            if(isnan(Mia(1)))
                Mia=[0;0;0];
            end
        end
    end
end