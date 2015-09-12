tic;
robot = neato('sim');
while toc < 40
    [vl,vr] = moveInArc(1,.3,.15);
    robot.sendVelocity(vl,vr);
    pause(.001);
end
robot.sendVelocity(0,0);
robot.shutdown();