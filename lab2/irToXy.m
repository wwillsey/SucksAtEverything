function [x y th] = irToXy( i, r )
% irToXy finds position and bearing of a range pixel endpoint
% Finds the position and bearing of the endpoint of a range pixel
% in the plane.
if i <= 180
    th = degtorad(i);
else
    th = degtorad(i - 360);
end

y = r * sind(i);
x = r * cosd(i);
end

