function [list, optList] = makeOVF(xc,yc,r_in,initTheta,decayFunc,legendName,list,optList)
%% Create vector field object using Goncalves method

%Radius = r_in./1E1
Radius = 10;
oVFa = VectorField('Gradient',Radius);
oVFa.G=0;  % Convergence weight
oVFa.H=0;%5;   % Circulation weight
oVFa.L=0;   % Time varying weight
if initTheta < 0
    oVFa.H = oVFa.H;
end
if initTheta > 0
    oVFa.H = -oVFa.H;
end
    
oVFa.xc=xc;  % VF center x-component
oVFa.yc=yc;  % VF center y-component
oVFa.mLegendName = legendName;

%% VF plot options
oVF1.VF = oVFa; 
oVF1.plotcenter = [oVFa.xc,oVFa.yc];
oVF1.plotrange = Radius.*3;
oVF1.plotradius = Radius;
bShowVectorField= true;
opt.bNormVFVectors = true;
opt.bPlotQuiverNorm = true;
opt.bShowCircle = true;
opt.CustomNumberOfPoints=25;
opt.DecayFunc = decayFunc;

%% Add VF to list of VFs to be plotted (how Jay creates multiple fields)
list = {list{:},oVF1};
optList = {optList{:}, opt};

end
