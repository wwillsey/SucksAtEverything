function [ vl, vr ] = moveInArc(x,y,v)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

if abs(y) < .01
    vr = v;
    vl = v;
else
    width = .235;
    R = (x^2 + y^2)/(2*y);
    vr = (v*width)/(2*R)+v;
    vl = v - (v*width)/(2*R);
    if v < 0
        tmp = vr;
        vr = vl;
        vl = tmp;
    end
end
end

