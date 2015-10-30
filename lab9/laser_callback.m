function [ output_args ] = laser_callback( src, msg )
%LASER_CALLBACK Summary of this function goes here
%   Detailed explanation goes here
global system ran 
if(system.terminated && ~ran)
    lidar_read = msg.Ranges;
    rImage = rangeImage(lidar_read, 1, true);
    [pts, th, best] = rImage.find_all_lines(0.01, 4);
    err = 0.0001;
    x = rImage.xArray(best(1));
    y = rImage.yArray(best(1));
    
    figure, rImage.plotXvsY(2);
    hold on;
    plot(-y, x, 'r*');
    th = best(2);
%     while(size(pts) == 0)
%         err = err*10;
%         [pts, th, best] = rImage.find_all_lines(err, 4);
%     end
    if(size(pts) == 0)
        disp('NO LINE FOUND');
    else

    %     th = 0;
        y_adjust = 0;
        if(x < 0)
            if(y > 0)
                y_adjust = 0.05;
            end
            if(y < 0)
                y_adjust = -0.05;
            end
        end
        T_og = [1,0,-17/100; 0,1,y_adjust;0,0,1]; %goal pose in terms of object
        T_so = [cos(th), -sin(th), x; sin(th), cos(th), y; 0,0,1];
        T_rs = [1,0,-0.085;0,1,0;0,0,1];
        T_final = T_rs*T_so*T_og;
        T_final(1,3)
        T_final(2,3)
        robot_trajectory = cubicSpiral.planTrajectory(T_final(1,3), T_final(2,3), th, 1);
        %robot_trajectory = cubicSpiral.planTrajectory(-60/100, 0, -pi, 1);
        robot_trajectory.planVelocities(0.2);
        system.trajectoryFollower.loadTrajectory(robot_trajectory, [0,0,0]);
        system.terminated = false;
        ran = true;
    end
end
end

