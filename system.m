function [F] = system(h,v,f,ts)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
k1 = f(h, v); 
k2 = f(h + ts/2*k1, v);
k3 = f(h + ts/2*k2, v); 
k4 = f(h + ts*k3, v); 

F=full(ts/6*(k1 +2*k2 +2*k3 +k4));
end

