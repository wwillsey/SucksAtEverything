function  encoder_callback( handle, event )
%ENCODER_CALLBACK Summary of this function goes here
%   Detailed explanation goes here
global robot;
global tp;
global terminated current running;
global feed_back_plot1;
global system;
global robot_trajectory2 robot_trajectory3;
global stop_timer;
global data_log;

%reading raw data
el_n = double(handle.LatestMessage.Left)/1e3;
er_n = double(handle.LatestMessage.Right)/1e3;
if ~system.terminated
    if isempty(tp) || tp == -1
        %Initialize the first iteration
        tp = double(handle.LatestMessage.Header.Stamp.Nsec) / 1e9;
        system.update(el_n, er_n, 0)
    else
        %% updating times and real time poses
        t_now = double(handle.LatestMessage.Header.Stamp.Nsec) / 1e9;
        dt = t_now - tp;
        if(dt < 0);
            dt = 1 + t_now - tp;
        end
        tp = t_now;
        system.update(el_n, er_n, dt);
      
        %% Plotting for debugging 
        if(running)
            currentPose = system.estRobot.robot_pose;
            referencePose = system.trajectoryFollower.robot_trajectory.getPoseAtTime(system.t_traj);
            referencePose(3) = 1;
            refPose = system.trajectoryFollower.T * referencePose;
            refPose(1) = refPose(1) / refPose(3);
            refPose(2) = refPose(2) / refPose(3);
            formatSpec = 'time: %f  x: %f  y: %f refX: %f refY: %f \n';
            fprintf(data_log, formatSpec, system.t_accum, refPose(1), refPose(2), referencePose(1), referencePose(2));
            if(current == 1)
                feed_back_plot1.update_plot(currentPose(1), currentPose(2), refPose(1), refPose(2));
            end
        end
         
        %% execute the movements
        if(system.trajectoryFollower.finished == true)
            if(stop_timer < 0)
                stop_timer = system.t_accum;
                running = false;
            elseif(system.t_accum - stop_timer > 5)
                running = true;
                stop_timer = -1;
                system.t_traj = 0;
                if(system.count == 0)
                    system.trajectoryFollower.loadTrajectory(robot_trajectory2, system.trajectoryFollower.robot_trajectory.getPoseAtTime(system.t_accum));
                elseif (system.count == 1)
                    system.trajectoryFollower.loadTrajectory(robot_trajectory3, system.trajectoryFollower.robot_trajectory.getPoseAtTime(system.t_accum));
                end
                system.count = system.count + 1;
            end
        end
      
        system.executeTrajectory();
    end
else
    robot.sendVelocity(0,0);
end

end

