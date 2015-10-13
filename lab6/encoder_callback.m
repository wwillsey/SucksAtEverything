function  encoder_callback( handle, event )
%ENCODER_CALLBACK Summary of this function goes here
%   Detailed explanation goes here
global robot;
global robot_trajectory1 robot_trajectory2 robot_trajectory3 trajectory_follower fbackcontrol;
global t_accum tp running;
global feed_back_plot error_plot;
global terminate current;
global  feed_back_plot1 feed_back_plot2 feed_back_plot3;
use_feedback = true;

%reading raw data
el_n = double(handle.LatestMessage.Left)/1e3;
er_n = double(handle.LatestMessage.Right)/1e3;

if ~terminate
    if isempty(tp) || tp == -1
        %Initialize the first iteration
        tp = double(handle.LatestMessage.Header.Stamp.Nsec) / 1e9;
        fbackcontrol.init_state(el_n, er_n)
    else
        %% updating times and real time poses
        t_now = double(handle.LatestMessage.Header.Stamp.Nsec) / 1e9;
        dt = t_now - tp;
        if(dt < 0);
            dt = 1 + t_now - tp;
        end
        tp = t_now;
        t_accum = t_accum + dt;
        fbackcontrol.update_pose(el_n, er_n, dt); %Get where the robot is
        %% Plotting for debugging 
        if(running)
            currentPose = fbackcontrol.robot_pose;
            referencePose = trajectory_follower.robot_trajectory.getPoseAtTime(t_accum);
            if(current == 1)
                feed_back_plot1.update_plot(currentPose(1), currentPose(2), referencePose(1), referencePose(2));
            elseif current == 2
                feed_back_plot2.update_plot(currentPose(1), currentPose(2), referencePose(1), referencePose(2));
            elseif current == 3
                feed_back_plot3.update_plot(currentPose(1), currentPose(2), referencePose(1), referencePose(2));
            end
            %error_plot.update_plot(t_accum, fbackcontrol.error(1), t_accum, fbackcontrol.error(2));
        end
        
        %% execute the movements
%       [vl, vr] = trajectory_follower.getVelocity(t_accum, use_feedback);
%       robot.sendVelocity(vl, vr);
        
        executeTrajectory(t_accum, use_feedback);
    end
else
    robot.sendVelocity(0,0);
end

end

