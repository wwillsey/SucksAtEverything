function  laser_callback( src, msg)
%CALLBACK Summary of this function goes here
%   Detailed explanation goes here
<<<<<<< HEAD
nearest_obj = lidar_reading(msg);
velocity_to_robot(nearest_obj);
pause(0.01);
% if(res(1) > 1)
%     vel = min((res(1)-1), 0.15);
% else
%     vel = max(res(1)-1, -0.15);
% end
=======
msg.Header.Stamp
res = lidar_reading(msg);
pause(0.001);
if(res(1) > 1.04)
    vel = min((res(1)-1), 0.15);
    robot.sendVelocity(.03, .03);
elseif (res(1) < 0.96)
    vel = max(res(1)-1, -0.15);
    robot.sendVelocity(-.03, -.03);
else
    robot.sendVelocity(0, 0);
end
>>>>>>> 49c677aaefb08efa9c28dd0cc49a20d44964dbbd

% pause(0.001)
end

