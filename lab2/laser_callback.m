function  laser_callback( src, msg, robot)
%CALLBACK Summary of this function goes here
%   Detailed explanation goes here

nearest_obj = lidar_reading(msg);  %get the nearest object according to laser
[vl, vr] = velocity_to_robot(nearest_obj); %get the velocity for the robot
robot.sendVelocity(vl, vr);
pause(0.01);
end

