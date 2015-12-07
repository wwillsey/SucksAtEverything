function  encoder_callback( handle, event )
%ENCODER_CALLBACK Summary of this function goes here
%   Detailed explanation goes here
global robot;
global tp;
global terminated current running;
global feed_back_plot1;
global system;

%this shit needs to go LOL
global robot_trajectory1 robot_trajectory2 robot_trajectory3;
global stop_timer;
global waiting acquiring;
global acq_q drop_q;
global use_localization;
global rotate_first;
%reading raw data
el_n = double(handle.LatestMessage.Left)/1e3;
er_n = double(handle.LatestMessage.Right)/1e3;
if ~system.terminated && ~acquiring
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
        % i guess we can use the system.count to keep track of if we are
        % going to acquisition, pickup, or dropoff spot prolly easiest. we
        % might want like a global pose so we know where to go for the
        % acquisition shit
        if(system.trajectoryFollower.finished == true && rotate_first == false )
            if(~waiting)
            if(stop_timer < 0)
                stop_timer = system.t_accum;
                running = false;
  
            elseif(system.t_accum - stop_timer > 0.2)
                running = true;
                stop_timer = -1;
                if(system.count == 0)
                    disp('acquiring');
                    acquiring = true;
                elseif(system.count == 1)
                    use_localization = false;
                    robot.forksUp()
                    disp('go back');
                    cp = system.estRobot.robot_pose_fus;
                    h = [cos(cp(3)) -sin(cp(3)) cp(1); sin(cp(3)) cos(cp(3)) cp(2); 0 0 1];
                    fp = h^-1 * [0.75;0.25;1];
                    robot_trajectory2 = trapezoidaStepReferenceControl(0.25, 0.15, -0.15, [0,0,0]);
                    system.trajectoryFollower.loadTrajectory(robot_trajectory2, cp);
                elseif(system.count == 2)
                    disp('rotate');
                    use_localization = false;
                    cp = system.estRobot.robot_pose_fus;
                    h = [cos(cp(3)) -sin(cp(3)) cp(1); sin(cp(3)) cos(cp(3)) cp(2); 0 0 1];
                    robot_trajectory3 = turnReference(0.1, 0.12, pi, [0,0,0]);
                    system.trajectoryFollower.loadTrajectory(robot_trajectory3, cp);
                elseif(system.count == 3)
                    use_localization = false;
                    disp('go to drop off');
                    cp = system.estRobot.robot_pose_fus;
                    h = [cos(cp(3)) -sin(cp(3)) cp(1); sin(cp(3)) cos(cp(3)) cp(2); 0 0 1];
                    current_drop = drop_q(1, :);
                    current_drop(3) = 1;
                    fp = h^-1 * current_drop';
                    drop_q = drop_q(2:end, :);
                    robot_trajectory1 = cubicSpiral.planTrajectory(fp(1), fp(2), -pi/2-cp(3), 1);
                    robot_trajectory1.planVelocities(0.2);
                    system.trajectoryFollower.loadTrajectory(robot_trajectory1, cp);
                elseif(system.count == 4)
                    robot.forksDown()
                    disp('go back');
                    cp = system.estRobot.robot_pose_fus;
                    h = [cos(cp(3)) -sin(cp(3)) cp(1); sin(cp(3)) cos(cp(3)) cp(2); 0 0 1];
                    fp = h^-1 * [0.75;0.25;1];
                    robot_trajectory2 = trapezoidaStepReferenceControl(0.25, 0.15, -0.1, [0,0,0]);
                    system.trajectoryFollower.loadTrajectory(robot_trajectory2, cp);
                elseif(system.count == 5)
                    disp('rotate');
                    use_localization = true;
                    cp = system.estRobot.robot_pose_fus;
                    h = [cos(cp(3)) -sin(cp(3)) cp(1); sin(cp(3)) cos(cp(3)) cp(2); 0 0 1];
                    fp = h^-1 * [0.75;0.25;1];
                  %  if size(acq_q,1) > 4
                  %      robot_trajectory3 = turnReference(0.10, 0.12, pi/2+.1, [0,0,0]);
                  %  else
                        robot_trajectory3 = turnReference(0.10, 0.12, pi, [0,0,0]);
                  %  end
                    system.trajectoryFollower.loadTrajectory(robot_trajectory3, cp);
                    system.count = -2;
                    waiting = true;
                    stop_timer = system.t_accum;
                end
                system.t_traj = 0;
                system.count = system.count + 1;
            end
            else
                if(system.t_accum - stop_timer > 5)
                    system.count = 6;
                    system.terminated = true;
                    stop_timer = -1;
                    waiting = false;
                    acquiring = false;
                end
            end
        elseif(system.trajectoryFollower.finished == true && rotate_first == true)
            rotate_first = false;
            system.terminated = true;
            return;
        end
        if(~acquiring)
            system.executeTrajectory();
        end
    end
else
    if(size(acq_q ,1 ) > 0 && acquiring == false)
       disp('planing');
        use_localization = true;
        
        lidar_read = robot.laser.LatestMessage.Ranges;
        rImage = rangeImage(lidar_read, 10, true);
        x = rImage.xArray;
        y = rImage.yArray;
        pts = [x;y;ones(1, rImage.numPix)];
        [succ, p_lid] = system.map.refinePose(pose(system.estRobot.robot_pose_fus), pts, 200);
        if(succ)
            disp('strongly refine pose');
            robot_pose = p_lid.getPoseVec'
            system.estRobot.robot_pose_fus = robot_pose;
        end

        
        cp = system.estRobot.robot_pose_fus;
        h = [cos(cp(3)) -sin(cp(3)) cp(1); sin(cp(3)) cos(cp(3)) cp(2); 0 0 1];
        acq_current = acq_q(1, :);
        pick_up_ang = acq_current(3);
        acq_current(3) = 1;
        fp = h^-1 * acq_current';
        acq_q = acq_q(2:end, :);
        robot_trajectory1 = cubicSpiral.planTrajectory(fp(1), fp(2), pick_up_ang-cp(3), 1);
        robot_trajectory1.planVelocities(0.15);
        system.trajectoryFollower.loadTrajectory(robot_trajectory1, cp);
        system.t_traj = 0;
        acquiring = false;
        system.terminated = false;
        system.count = 0;
    end
    robot.sendVelocity(0,0);
end

end

