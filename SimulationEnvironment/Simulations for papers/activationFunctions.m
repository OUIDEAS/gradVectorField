


function p = activationFunctions(theta,type)


for i=1:length(theta)
if type =='r'
    if theta(i)<deg2rad(90)
        p(i) = 1/2.*tanh(30.*theta(i)-pi)+1;
    else
        p(i) = -1/2.*tanh(5.*theta(i)-5*pi)+1;
    end

elseif type =='a'
        if theta(i)<deg2rad(90)
        p(i) = -1/2.*tanh(30.*theta(i)-pi)+1;
    else
        p(i) = 1/2.*tanh(5.*theta(i)-15*pi)+1;
        end
        
elseif type == 'g'
   
    if theta(i)<deg2rad(180)
        p(i) = -1/2.*tanh(3.*theta(i)-pi)+0.5+0.5;
    else
        p(i) = 0;%+1/2.*tanh(5.*theta(i)-4*pi)+0.5;
    end
    
end
end
    


end



% clc
% clear
% close all
% 
% theta = 0:0.01:2*pi;
% 
% 
% 
% 
% 
% for i=1:length(theta)
% 
%     if theta(i)<deg2rad(180)
%         p(i) = -1/2.*tanh(2*theta(i)-pi)+0.5;
%     else
%         p(i) = 0
%     end
% end
% 
% plot(rad2deg(theta),p)



