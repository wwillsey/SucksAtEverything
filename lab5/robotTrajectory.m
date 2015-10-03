classdef robotTrajectory 
    %UNTITLED3 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        times;
        trajectory;
    end
    
    methods
        function obj = robotTrajectory(ref, time_start, time_end, init_pose)
            dt = 0.01
            x = init_pose(1);
            y = init_pose(2);
            theta = init_pose(3);
            obj.times = zeros(ceil(time_end - time_start) / dt, 1);
            obj.trajectory = zeros(ceil(time_end - time_start) / dt, 4);
            index = 1;
            for t = time_start:dt:time_end
                [V, w] = ref.computeControl(t);
                theta = theta + w*dt;
                dx = cos(theta) * V*dt;
                dy = sin(theta) * V*dt;
                x = x+dx;
                y = y+dy;
                obj.trajectory(index, :) = [x, y, theta, V]; 
                obj.times(index) = t;
                index = index + 1;
            end
        end
        function pose = getPoseAtTime(obj, t)
            all_x = obj.trajectory(1,:);
            all_y = obj.trajectory(2,:);
            all_theta = obj.trajectory(3,:);
            x_q = interp1(obj.times, all_x, t);
            y_q = interp1(obj.times, all_y, t);
            theta_q = interp1(obj.times, all_theta, t);
            pose = [x_q, y_q, theta_q];
        end
    end
    
end

