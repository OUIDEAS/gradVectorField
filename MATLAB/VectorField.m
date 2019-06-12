classdef VectorField
    properties
        G;
        H;
        L;
        bUseVRel=~true;
        
        vel_x=0;
        vel_y=0;
        vel_z=0;
        
        psi = .25;
        gam = .25;
        lam = 1;
        K = 1;
        ZStart = 0;
        
        mCircleRadius;
        
        xc;
        yc;
        zc;
        
        xc_history=[];
        yc_history=[];
        zc_history=[];
        bUsePathFunc=false;
        velPathFunc=[];
        bGradientVF = false;
        bLyapunovVF = false;
        bStraightVF = false;
        bSpiralVF   = false;
        
        radFunc=[];
        mLegendName;
    end
    
    methods
        function obj = VectorField(type,radius_in)
            if(strcmp(type,'Gradient'))
                obj.bGradientVF=true;
            elseif(strcmp(type,'Lyapunov'))
                obj.bLyapunovVF=true;
            elseif(strcmp(type,'Straight'))
                obj.bStraightVF=true;
            elseif(strcmp(type,'Spiral'))
                obj.bSpiralVF = true;
            else
                error('not a known type')
            end
            obj.mCircleRadius=radius_in;
        end
        function V = GetVF_at_XY(obj,si)
            
            s.x = si.x;
            s.y = si.y;
            s.z = si.z;

            s.r = obj.mCircleRadius;
            
            s.psi   = obj.psi;
            s.gam   = obj.gam;
            s.lam   = obj.lam;
            s.K     = obj.K;
            
            s.uav_vy = si.uav_vy;
            s.uav_vx = si.uav_vx;
            s.uav_vz = si.uav_vz;
            
            s.G=obj.G;
            s.H=obj.H;
            s.L=obj.L;
            s.bNormVFVectors=si.bNormVFVectors;
            
            s.bUseVRel=obj.bUseVRel;
            s.xc=obj.xc;
            s.yc=obj.yc;
            s.zc=obj.zc;
           
            
            if(~obj.bUsePathFunc || isempty(obj.velPathFunc))
                s.velx=obj.vel_x;
                s.vely=obj.vel_y;
                s.velz=obj.vel_z;
            else
                vel_v = obj.velPathFunc(si.t);
                s.velx=vel_v(1);
                s.vely=vel_v(2);
                s.velz=vel_v(3);
            end
            s.r=obj.mCircleRadius;
            if(obj.bGradientVF)
                V = obj.VFtv(s);
            elseif(obj.bLyapunovVF)
                V.F = obj.VFLyapunov(s);
            elseif(obj.bStraightVF)
                V = obj.VFStraight(s);
            elseif(obj.bSpiralVF)
                V = obj.VFSpiral(s);
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
                obj.vel_z=vel_v(3);
            end
            obj.xc = obj.xc + obj.vel_x*dt;
            obj.yc = obj.yc + obj.vel_y*dt;
            obj.zc = obj.zc + obj.vel_z*dt;
            
            obj.xc_history=[obj.xc_history;obj.xc];
            obj.yc_history=[obj.yc_history;obj.yc];
            obj.zc_history=[obj.zc_history;obj.zc];
            
            if(~isempty(obj.radFunc))
                vel_r = sqrt(uavv.x^2+uavv.y^2+uavv.z^2) / sqrt(obj.vel_x^2+obj.vel_y^2+obj.vel_z^2);
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
            obj.zc = newxy(3);
            
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
            if(isfield(opt,'CustomCylinderHeight'))
                height = opt.CustomCylinderHeight;
            end
            if(isfield(opt,'bCustomCenter'))
                xct = opt.bCustomCenter(1);
                yct = opt.bCustomCenter(2);
                zct = opt.bCustomCenter(3);
                
            else
                xct = obj.xc;
                yct = obj.yc;
                zct = obj.zc;
            end
            VFUAV = [];
            if(isfield(opt,'UAV'))
                VFUAV = opt.UAV;
            end
            x_list = linspace(-limit,limit,NumPoints) + xct;
            y_list = linspace(-limit,limit,NumPoints) + yct;
            z_list = linspace(-2*height,2*height,NumPoints) + zct;
    
            zero   = linspace(0,0,NumPoints);
            
            k=1;
            Vect=[];
            
            for i=1:length(x_list)
                x = x_list(i);
                for ii=1:length(y_list)
                    y=y_list(ii);
                    s.xc=obj.xc;
                    s.yc=obj.yc;
                    s.zc=obj.zc;
                    
                    s.psi   = obj.psi;
                    s.gam   = obj.gam;
                    s.lam   = obj.lam;
                    s.K     = obj.K;
                    s.ZStart = obj.ZStart;
                    s.r=obj.mCircleRadius; 
                    
                    
                    s.x = x;
                    s.y = y;

                    A = - s.r*(tan(s.gam)/s.lam);
                    theta = pi;
                    shift  =  A * (-theta);
                    change =  A * ( theta);

                    s.xc=obj.xc;
                    s.yc=obj.yc;
                    s.zc=obj.zc;
                                        
                    x_coil(i,ii) = x_list(i);
                    y_coil(i,ii) = y_list(ii);
                    
                    for turns = 1:s.K
                        z_coil(i,ii,turns) = A * (atan2((y_coil(i,ii)-s.yc),(x_coil(i,ii)-s.xc)) - s.psi) + s.zc - shift * (turns-1)*2;                       
                    end
                    
                    for iii = 1:length(z_list)
                        z = z_list(iii);

                        s.x = x;
                        s.y = y;
                        s.z = z;
                        
                        s.xc=obj.xc;
                        s.yc=obj.yc;
                        s.zc=obj.zc;
                       
                        s.psi   = obj.psi;
                        s.gam   = obj.gam;
                        s.lam   = obj.lam;
                        
                        s.velx=obj.vel_x;
                        s.vely=obj.vel_y;
                        s.velz=obj.vel_z;
                          
                        r_at_now = sqrt((x-obj.xc)^2+(y-obj.yc)^2);
                        Rxx(i,ii,iii) = r_at_now;
                        P(i,ii,iii)=1;
                        
                        if(isfield(opt,'DecayFunc'))
                            P(i,ii,iii) = opt.DecayFunc(r_at_now);
                        end
                        s.G=obj.G;
                        s.H=obj.H;
                        s.L=obj.L;
                        if(~isempty(VFUAV))
                            s.bNormVFVectors=VFUAV.bNormVFVectors;
                            uav_vel = VFUAV.GetVelocityV();
                            s.uav_vx=uav_vel(1);
                            s.uav_vy=uav_vel(2);
                            s.uav_vz=uav_vel(3);
                        else
                            s.bNormVFVectors=opt.bNormVFVectors;
                            s.uav_vx=0;
                            s.uav_vy=0;
                            s.uav_vz=0;
                        end
                        if(obj.bGradientVF)
                            VF_res = obj.VFtv(s);%VFtv(x,y,0,xc,yc,vel_t,0,r,G,H);
                            VF_list_tv{i,ii,iii}=VF_res.tv;
                            VF_list_circ{i,ii,iii}=VF_res.circ;
                            VF_list_conv{i,ii,iii}=VF_res.conv;
                            VF_list{i,ii,iii}=VF_res.F;%VF_res.F;
                            
                        elseif(obj.bLyapunovVF)
                            VF_res = obj.VFLyapunov(s);
                            VF_list{i,ii,iii} = VF_res;
                            
                        elseif(obj.bStraightVF)
                            VF_res = obj.VFStraight(s);
                            VF_list_tv{i,ii,iii}=VF_res.tv;
                            VF_list_circ{i,ii,iii}=VF_res.circ;
                            VF_list_conv{i,ii,iii}=VF_res.conv;
                            VF_list{i,ii,iii} = VF_res.F;
                            
                        elseif(obj.bSpiralVF)
                            VF_res = obj.VFSpiral(s);
                            VF_list_tv{i,ii,iii}=VF_res.tv;
                            VF_list_circ{i,ii,iii}=VF_res.circ;
                            VF_list_conv{i,ii,iii}=VF_res.conv;
                            VF_list{i,ii,iii} = VF_res.F;
                            
                        end
                        
                        u(i,ii,iii)=VF_list{i,ii,iii}(1);
                        v(i,ii,iii)=VF_list{i,ii,iii}(2);
                        w(i,ii,iii)=VF_list{i,ii,iii}(3);
                          
                        heading(i,ii,iii) =   atan2(v(i,ii,iii),u(i,ii,iii));
                        pitchVF(i,ii,iii) =   atan2(sqrt(u(i,ii,iii).^2+v(i,ii,iii).^2),w(i,ii,iii))-pi/2;
     
                        XCom(i,ii,iii) = cos(heading(i,ii,iii))*cos(pitchVF(i,ii,iii));
                        YCom(i,ii,iii) = sin(heading(i,ii,iii))*cos(pitchVF(i,ii,iii));
                        ZCom(i,ii,iii) = -sin(pitchVF(i,ii,iii));
                        
                        x_q(i,ii,iii)=x+s.xc;
                        y_q(i,ii,iii)=y+s.yc;
                        z_q(i,ii,iii)=z+s.zc;
                        
                        zero(i,ii,iii) = zero(iii);
                        
                    end
                end
            end
            if(obj.bGradientVF)
                for i=1:length(x_list)
                    for ii=1:length(y_list)
                        for iii = 1:length(z_list)
                        ucirc(i,ii,iii)=VF_list_circ{i,ii,iii}(1);
                        vcirc(i,ii,iii)=VF_list_circ{i,ii,iii}(2);
                        wcirc(i,ii,iii)=VF_list_circ{i,ii,iii}(3);

                        uconv(i,ii,iii)=VF_list_conv{i,ii,iii}(1);
                        vconv(i,ii,iii)=VF_list_conv{i,ii,iii}(2);
                        wconv(i,ii,iii)=VF_list_conv{i,ii,iii}(3);

                        utv(i,ii,iii)=VF_list_tv{i,ii,iii}(1);
                        vtv(i,ii,iii)=VF_list_tv{i,ii,iii}(2);
                        end
                    end
                end
            end
            
            %bPlotQuiverNorm=false;
            if(opt.bPlotQuiverNorm)
                normnorm = sqrt(u.^2+v.^2+w.^2);
                u=u./sqrt(u.^2+v.^2+w.^2);
                v=v./sqrt(u.^2+v.^2+w.^2);
                w=w./sqrt(u.^2+v.^2+w.^2);
%                 
%                 u=u./normnorm;
%                 v=v./normnorm;
                %VectNorm = Vect./norm(Vect,2);
                if(obj.bGradientVF)
%                     circ_norm = sqrt(ucirc.^2+vcirc.^2);
                    ucirc=ucirc./sqrt(ucirc.^2+vcirc.^2+wcirc.^2);
                    vcirc=vcirc./sqrt(ucirc.^2+vcirc.^2+wcirc.^2);
                    wcirc=wcirc./sqrt(ucirc.^2+vcirc.^2+wcirc.^2);
                    
%                     conv_norm = sqrt(uconv.^2+vconv.^2);
                    uconv=uconv./sqrt(uconv.^2+vconv.^2+wconv.^2);
                    vconv=vconv./sqrt(uconv.^2+vconv.^2+wconv.^2);
                    wconv=wconv./sqrt(uconv.^2+vconv.^2+wconv.^2);
                    
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
                    for iii = 1:length(z_list)
                        z = z_list(iii);
                        nu(i,ii,iii) = u(i,ii,iii) .* P(i,ii,iii);
                        nv(i,ii,iii) = v(i,ii,iii) .* P(i,ii,iii);
                        nw(i,ii,iii) = w(i,ii,iii) .* P(i,ii,iii);

                        Q(i,ii,iii) = sqrt(u(i,ii,iii)^2+v(i,ii,iii)^2+w(i,ii,iii)^2);
                        QP(i,ii,iii) = sqrt(nu(i,ii,iii)^2+nv(i,ii,iii)^2+nw(i,ii,iii)^2);
                        
                    end
                end
            end
            %only for plotting...
            if(isfield(opt,'sOnlyShow'))
                if(strcmp(opt.sOnlyShow, 'Circ'))
                    nu = ucirc;
                    nv = vcirc;
                    nw = wcirc;
                elseif(strcmp(opt.sOnlyShow, 'Conv'))
                    nu = uconv;
                    nv = vconv;
                    nw = wconv;
                elseif(strcmp(opt.sOnlyShow, 'TV'))
                    nu = utv;
                    nv = vtv;
                end
            end
            
            %%%%%% PLOTTING %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            hold on
            
            if opt.bShowSurfaces == true
                if(isfield(opt,'bShowCircle'))
                    th = linspace(0,2*pi,length(x_list));
                    x_c=obj.xc;
                    y_c=obj.yc;
                    z_c=obj.zc;
                    Rc = obj.mCircleRadius;
                    if(isfield(opt,'bCustomCircleRadiusPlot'))
                        Rc =  opt.bCustomCircleRadiusPlot;
                    end

                    if(isfield(opt,'bCustomCylinderHeight'))
                        Height = opt.bCustomCylinderHeight;
                    end

                    xunit = Rc * cos(th) + x_c;
                    yunit = Rc * sin(th) + y_c;
                    zunit = linspace(-s.K*height,height,length(x_list)) + z_c;

                    for xk = 1:length(x_list)
                        for theta = 1:length(th)
                            xunit3(xk,theta) = xunit(xk);
                            yunit3(xk,theta) = yunit(xk);
                            zunit3(xk,theta) = zunit(theta);
                        end
                    end
                        for data =  1:s.K
                            surf(x_coil,y_coil,z_coil(:,:,data))
                        end
                        cylinder = surf(xunit3,yunit3,zunit3);
                        alpha 0.25
                        shading interp
                        set(cylinder,'FaceColor',[1 0 0])
                    end 
                end
               
                %%%%%%%%%%%%% PLOTTING 3D %%%%%%%%%%%%%%%%%%%%%%%%%%
                %% Vector Fields
                
               if opt.bPlotVF == true
                    RET_H = quiver3(x_q,y_q,z_q,nu,nv,nw,1,'b');
                        if(~isempty(obj.mLegendName))
                            set(RET_H,'DisplayName',obj.mLegendName);
                        end
               end
            
            RET.QP = QP;
            RET.Q = Q;
            RET.X = x_q;
            RET.Y = y_q;
            RET.Z = z_q;
            RET.P = P;
            RET.R = Rxx;
            
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function V = VFStraight(obj,s)
            Vcnv=s.G.*[0;s.y;s.z];
            Vcrc=s.H.*[1;0;0];
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
        function alp1 = alpha1_cylinder(o,s) 
            alp1 = (s.x-s.xc)^2+(s.y-s.yc)^2-s.r^2;
        end
        function alp2 = alpha2_plane(o,s)
            alp2 = s.z;
        end
        
        function V = Vconv_c(o,s) 
            V1 = -o.alpha1_cylinder(s).*[2.*(s.x-s.xc);2.*(s.y-s.yc);0]./s.r;%^2;
            V2 = o.alpha2_plane(s)*[0;0;1];
            V = V1+V2;

        end
        function V = Vcirc_c(o,s) 
            V = [2.*(s.y-s.yc);-2.*(s.x-s.xc);0];            
        end 
            
        %% Spiral %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function V = Vcirc_spiral(o,s)
            x =  (s.x - s.xc);
            y =  (s.y - s.yc);
            zs = (s.z - s.zc);
            
            A = - s.r*(tan(s.gam)/s.lam);
            theta = pi;
            shift  =  A * (-theta);
            change =  A * ( theta);
            
            for turns = 1:s.K
                 z(turns)  = A * (atan2(y,x) - s.psi) - shift * (turns-1) * 2;
            end  
            
            V1 = - 2*y/s.r^3;
            V2 =   2*x/s.r^3;
            
            V3 = - (2 * tan(s.gam)) / ((s.r^2 * ((y.^2/x.^2) + 1))*s.lam) - (2 * y.^2 * (tan(s.gam)))/(s.r^2 *s.lam* x.^2 * ((y.^2/x.^2) + 1));
            
            V = [V1;V2;V3];   
        end
        
        function V = Vconv_spiral(o,s)
            x = (s.x - s.xc);
            y = (s.y - s.yc);
            
            A = - s.r*(tan(s.gam)/s.lam);
            theta = pi;
            shift  =  A * (-theta);
            change =  A * ( theta);
            
            for turns = 1:s.K
                 z(turns)  = A * (atan2(y,x) - s.psi) - shift * (turns-1) * 2;
            end  
            
            SIG1 = ((z/s.r) - (tan(s.gam) * ((s.psi - atan2(y,x))/(s.lam)))) ;
            SIG2 = ((x/s.r).^2 + (y/s.r).^2 - 1);

            DXT = (4 * x * SIG2/(s.r.^2)) - (2 * y * tan(s.gam)*SIG1)/(s.lam*(x).^2 * (((y).^2/(x).^2) + 1));
            DYT = (4 * y * SIG2/(s.r.^2)) + (2 * tan(s.gam)*SIG1)/(s.lam * x * (((y).^2/(x).^2)+1));
            DZT = 2*SIG1/s.r;
            
            V   = [DXT;DYT;DZT];   
        end
         %% END Spiral %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        
       function V=VFSpiral(o,s)
            Vcnv = s.G.*o.Vconv_spiral(s);
            Vcrc = s.H.*o.Vcirc_spiral(s);
            TV   = 0;
                
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