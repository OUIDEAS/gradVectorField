function [deviation] = gust(UAVX,UAVY,WindX,magnitude)

b1 = WindX - 100;
b2 = WindX + 100;

c = 10;

deviation = magnitude * (tanh((UAVX-b1)/c)-tanh((UAVX-b2)/c))/2 + UAVY;
   
end
