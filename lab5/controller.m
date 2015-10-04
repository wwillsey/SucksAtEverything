classdef controller
    %UNTITLED2 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
    end
    
    methods
        function obj = controller()
        end
        
        function [vl, vr] = getVel(obj, traj, robot, actual_pose)
            kpx = .5;
            kpy = .5;
            
            goal_pose = traj.getPoseAtTime(t);
            error = goal_pose - actual_pose;
            u = 0;
            t_now = double(handle.LatestMessage.Header.Stamp.Nsec) / 1e9;
            error_prev = error;
            if(tp == -1)
                tp = t_now;
                error = goal_state - actual_state;
                error_prev = error;
            else
                error = (goal_state - actual_state);
                max = .25;

                %Actual feed back loop
                ep = error *  kp;
                ed = ((error - error_prev) / dt) * kd; 
                ei = (ei + error * dt) * ki;
                u = ep + ed + ei;
                if u > max
                    u = max;
                elseif u < -max
                    u = -max;
                end
    
                if  abs(error) <= .0005
                    u = 0;
                end
    
            end
        end
    end
    
end

