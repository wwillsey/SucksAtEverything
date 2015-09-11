function [ ranges ] = nearestObject( robot )
%UNTITLED6 Summary of this function goes here
%   Detailed explanation goes here
maxobjectrange = 1.5;

ranges = robot.laser.LatestMessage.Ranges([1:45 316:360]);
res = robot.laser.LatestMessage.Ranges(1)
ranges = arrayfun(@(i) irToXy(i), ranges,'un');
ranges = ranges(cellfun(@(x) x(1)*x(1)  + x(2)*x(2),ranges) < maxobjectrange);

end

