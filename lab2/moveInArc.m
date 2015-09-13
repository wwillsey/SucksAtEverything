function [ vl, vr ] = moveInArc(x,y,v)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

if abs(y) < .01
    vr = v;
    vl = v;
else
    width = .235;
    R = ((x^2 + y^2)/(2*y));
    R = R*.5;
    vr = (v*width)/(2*R)+v;
    vl = v - (v*width)/(2*R);
    if v < 0
        tmp = vr;
        vr = vl;
        vl = tmp;
    end
end
if max(abs(vl),abs(vr)) >= 0.25
    vl = (vl / max(abs(vl),abs(vr))) * 0.25;
    vr = (vr / max(abs(vl),abs(vr))) * 0.25;
end
if min(abs(vl),abs(vr)) <= 0.03
    vl = (vl / min(abs(vl),abs(vr))) * 0.03;
    vr = (vr / min(abs(vl),abs(vr))) * 0.03;
end
end

