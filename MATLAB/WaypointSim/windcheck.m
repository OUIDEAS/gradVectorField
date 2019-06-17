clc;clear;close all
X = linspace(-300,300,300);
for i = 1:length(X)
    wind(i) = gust(X(i),0,0,0.5);
end
plot(X,wind)
grid on
xlabel('X (m)')
ylabel('Y (m)')
ylim([0,max(wind)+max(wind)*0.2]);

