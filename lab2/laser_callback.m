function  laser_callback( src, msg, robot)
%CALLBACK Summary of this function goes here
%   Detailed explanation goes here
nearest_obj = lidar_reading(msg);
max_vel = .2;
x = nearest_obj(1);
y = nearest_obj(2);
d = sqrt(x^2+y^2);
vel = (d - .7);
if abs(d-1) < .03
    vel = 0;
end
%v = rossubscriber('/vel');
%vel_prev = sqrt((double(v.LatestMessage.Left)/1000)^2 + ...
%    (double(v.LatestMessage.Right)/1000)^2);
[vl, vr] = moveInArc(x,y,vel);
robot.sendVelocity(double(vl), double(vr));
pause(0.01);
% if(res(1) > 1)
%     vel = min((res(1)-1), 0.15);
% else
%     vel = max(res(1)-1, -0.15);
% end


% pause(0.001)
end

