function executeTrajectory(time, use_feedback)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
global robot;
global robot_trajectory1 robot_trajectory2 robot_trajectory3 fbackcontrol;
global trajectory_follower;
global t_accum tp running;
global terminate;
persistent current;
if isempty(current)
    current = 1;
end
if(running)
    [vl, vr] = trajectory_follower.getVelocity(t_accum, use_feedback);
    robot.sendVelocity(vl, vr);
else
    robot.sendVelocity(0,0);
    
    if(toc > 5)
        running = true;
        global_reset;
    end
end
if(trajectory_follower.finished)
%     disp('finished');
    if(current == 1)
        trajectory_follower = trajectoryFollower(robot_trajectory2, fbackcontrol);
        current = 2;
    elseif(current == 2)
        trajectory_follower = trajectoryFollower(robot_trajectory3, fbackcontrol);
        current = 3;
    elseif (current == 3)
        terminate = true;
    end
    running = false;
    tic;
end
end

