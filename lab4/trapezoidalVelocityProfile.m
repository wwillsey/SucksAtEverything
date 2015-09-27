function [ uref ] = trapezoidalVelocityProfile( t, amax, vmax, dist, sgn )
%TRAPEZOIDALVELOCITYPROFILE Summary of this function goes here
%   Detailed explanation goes here
t_ramp = vmax / amax;
tf = (abs(dist) + vmax^2 / amax) / vmax;
if t < t_ramp && t > 0;
    uref = amax*t;
elseif (tf-t) < t_ramp && tf-t > 0 && t > 0
    uref = amax * (tf-t);
elseif t_ramp < t && t < tf - t_ramp && t > 0
    uref = vmax;
else
    uref = 0;
    uref = sgn*uref;
end

