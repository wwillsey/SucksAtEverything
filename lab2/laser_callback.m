function  laser_callback( src, msg, robot)
%CALLBACK Summary of this function goes here
%   Detailed explanation goes here

nearest_obj = lidar_reading(msg);  %get the nearest object according to laser
%[vl, vr] = velocity_to_robot(nearest_obj); %get the velocity for the robot
x = nearest_obj(1);
y = nearest_obj(2);
d = sqrt(x^2+y^2);
vel = (d - .9);
if abs(vel) < .03
    vel = 0;
end

[vl, vr] = moveInArc(x,y,vel);
vl = double(vl);
vr = double(vr);
if vl >= .3
    vl = .25
end
if vr >= .3
    vr = .25
end
if vl <= -.3
    vl = -.25
end
if vr <= -.3
    vr = -.25
end
robot.sendVelocity(vl, vr);
pause(0.01);
end

