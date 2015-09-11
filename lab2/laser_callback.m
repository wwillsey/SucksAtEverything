function  laser_callback( src, msg, robot )
%CALLBACK Summary of this function goes here
%   Detailed explanation goes here
res = lidar_reading(msg)
pause(0.001);
if(res(1) > 1)
    vel = min((res(1)-1), 0.15);
    robot.sendVelocity(.069, .069);
else
    vel = max(res(1)-1, -0.15);
    robot.sendVelocity(-.069, -.069);
end

pause(0.001)
end

