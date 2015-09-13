function  laser_callback( src, msg, robot)
%CALLBACK Summary of this function goes here
%   Detailed explanation goes here
nearest_obj = lidar_reading(msg);
x = nearest_obj(1);
y = nearest_obj(2);
d = sqrt(x^2+y^2);
%vel = (d - 1)*0.3;
if d < 1
    vel = -.15;
else
    vel = .15;
end
if abs(d-1) < .05
    vel = 0;
end

[vl, vr] = moveInArc(x,y,vel);
pause(0.01);
robot.sendVelocity(double(vl), double(vr));
pause(0.01);
% if(res(1) > 1)
%     vel = min((res(1)-1), 0.15);
% else
%     vel = max(res(1)-1, -0.15);
% end


% pause(0.001)
end

