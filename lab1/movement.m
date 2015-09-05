function movement( robot )
%MOVEMENT Summary of this function goes here
%   Detailed explanation goes here
sp_left = robot.encoders.LatestMessage.Left;
sp_right = robot.encoders.LatestMessage.Right;
c_left = sp_left;
c_right = sp_right;
while (c_left - sp_left) < 200 || (c_right - sp_right) < 200
    robot.sendVelocity(0.01, 0.01);
    pause(0.01);
    c_left=robot.encoders.LatestMessage.Left;
    c_right=robot.encoders.LatestMessage.Right;
end
robot.sendVelocity(0,0);
tic;
s = toc;
while (s) < 2
    robot.sendVelocity(0,0);
    pause(0.001);
    s = toc;
end

sp_left = robot.encoders.LatestMessage.Left;
sp_right = robot.encoders.LatestMessage.Right;
c_left = sp_left;
c_right = sp_right;
while (c_left - sp_left) > -200 || (c_right - sp_right) > -200
    robot.sendVelocity(-0.01, -0.01);
    pause(0.01);
    c_left=robot.encoders.LatestMessage.Left;
    c_right=robot.encoders.LatestMessage.Right;
end
robot.sendVelocity(0,0);

