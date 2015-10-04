classdef controller
    %UNTITLED2 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
    end
    
    methods
        function obj = controller()
        end
        
        function [V, w] = getVel(obj, traj, actual_pose, dt)
            kpx = .5;
            kpy = .5;
            
            goal_pose = traj.getPoseAtTime(t);
            error = goal_pose - actual_pose;
            V = kpx * error(1);
            w = kpy * error(2);

        end
    end
    
end

