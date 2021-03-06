function [ vl, vr ] = velocity_to_robot( nearest_obj )
%VELOCITY_TO_ROBOT Summary of this function goes here
%   Detailed explanation goes here

x = nearest_obj(1);
y = nearest_obj(2);
d = sqrt(x^2+y^2);
vel = (d - .9);
if abs(vel) < .03
    vel = 0;
end

[vl, vr] = moveInArc(x,y,vel);
vl = double(vl);
vr = double(vr);
end

