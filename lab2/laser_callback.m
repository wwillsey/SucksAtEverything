function  laser_callback( src, msg )
%CALLBACK Summary of this function goes here
%   Detailed explanation goes here
[x,y, b] = lidar_reading(msg);
pause(0.001); 
if(x > 1)
    vel = min((x-1), 0.15);
    robot.sendVelocity(vel, vel);
else
    vel = max(x-1, -0.15);
    robot.sendVelocity(vel, vel);
end
pause(0.001); 
end

