function movement( robot )

% Given the specific robot, the robot is able to move forward for 20cm
% pause for 2 seconds and move back for 20 cm
% Afterwards, matlab plots the actual distance traveled.  

sp_left = robot.encoders.LatestMessage.Left;
sp_right = robot.encoders.LatestMessage.Right;
origin = sp_left; %define origin for graph purpose.
c_left = sp_left;
c_right = sp_right;
timearray=[];
distarray=[];
tic;
accum = 1;
while (c_left - sp_left) < 200 || (c_right - sp_right) < 200
    robot.sendVelocity(0.05, 0.05);
    timearray(accum) = toc;
    distarray(accum) = c_left-sp_left;
    accum = accum+1;
    c_left=robot.encoders.LatestMessage.Left;
    c_right=robot.encoders.LatestMessage.Right;
    pause(0.001);
end
robot.sendVelocity(0,0);
s_begin = toc;
s_now = s_begin;
while (s_now-s_begin) < 2
    robot.sendVelocity(0,0);
    pause(0.001);
    s_now = toc;
    timearray(accum) = s_now;
    distarray(accum) = robot.encoders.LatestMessage.Left-origin;
    accum = accum+1;
end

sp_left = robot.encoders.LatestMessage.Left;
sp_right = robot.encoders.LatestMessage.Right;
c_left = sp_left;
c_right = sp_right;
while (c_left - sp_left) > -200 || (c_right - sp_right) > -200
    robot.sendVelocity(-0.05, -0.05);
    timearray(accum) = toc;
    distarray(accum) = c_left - origin;
    accum = accum+1;
    c_left=robot.encoders.LatestMessage.Left;
    c_right=robot.encoders.LatestMessage.Right;
    pause(0.001);
end
robot.sendVelocity(0,0);

plot(timearray, distarray);

