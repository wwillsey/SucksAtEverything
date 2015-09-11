function  laser_callback( src, msg, robot )
%CALLBACK Summary of this function goes here
%   Detailed explanation goes here
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

pause(0.001)
end

