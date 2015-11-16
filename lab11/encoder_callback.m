function  encoder_callback( handle, event )
%ENCODER_CALLBACK Summary of this function goes here
%   Detailed explanation goes here
global robot;
global tp;
global terminated current running;
global feed_back_plot1;
global system;
global robot_trajectory1 robot_trajectory2 robot_trajectory3;
global stop_timer;
global data_log;

%reading raw data
el_n = double(handle.LatestMessage.Left)/1e3;
er_n = double(handle.LatestMessage.Right)/1e3;
if ~system.terminated
    if isempty(tp) || tp == -1
        %Initialize the first iteration
        tp = double(handle.LatestMessage.Header.Stamp.Nsec) / 1e9;
        system.update(el_n, er_n, 0);
        disp('hi');
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
%         if(running)
%             currentPose = system.estRobot.robot_pose_fus;
%             referencePose = system.trajectoryFollower.robot_trajectory.getPoseAtTime(system.t_traj);
%             referencePose(3) = 1;
%             refPose = system.trajectoryFollower.T * referencePose;
%             refPose(1) = refPose(1) / refPose(3);
%             refPose(2) = refPose(2) / refPose(3);
%             formatSpec = 'time: %f  x: %f  y: %f refX: %f refY: %f \n';
%             fprintf(data_log, formatSpec, system.t_accum, refPose(1), refPose(2), referencePose(1), referencePose(2));
%             if(current == 1)
%                 feed_back_plot1.update_plot(currentPose(1), currentPose(2), refPose(1), refPose(2));
%                 
%             end
%         end
         
        %% execute the movements
        if(system.trajectoryFollower.finished == true)
            if(stop_timer < 0)
                stop_timer = system.t_accum;
                running = false;
            elseif(system.t_accum - stop_timer > 10)
                running = true;
                stop_timer = -1;
                if(system.count == 0)
                    cp = system.estRobot.robot_pose_fus
                    h = [cos(cp(3)) -sin(cp(3)) cp(1); sin(cp(3)) cos(cp(3)) cp(2); 0 0 1];
                    fp = h^-1 * [0.75;0.25;1];
                    robot_trajectory2 = trapezoidaStepReferenceControl(0.25, 0.25, -0.15);
                    system.trajectoryFollower.loadTrajectory(robot_trajectory2, cp);
                end
                system.t_traj = 0;
                system.count = system.count + 1;
            end
        end
        system.executeTrajectory();
    end
else
    robot.sendVelocity(0,0);
end

end

