classdef controller < handle
    %UNTITLED2 Summary of this class goes here
    %   Detailed explanation goes here
   
    properties
        robot_pose;
        robot_state;
        error;
    end
    
    methods(Access = public)
        function obj = controller()
        end  
        
        function [V, w] = linear_feedback(obj, refPose, estPose)
            %robot_trajectory act as refrobot
            kpx = 0.3;
            kpy = 0.3;
            goal_position = refPose(1:2);
            actual_position = estPose(1:2)';
            theta = estPose(3);
            error_world = goal_position - actual_position;
            error = [cos(theta), -sin(theta); sin(theta), cos(theta)]^-1 * error_world;
            adjust = [kpx, 0; 0, kpy] * error;
            V = adjust(1);
            w = adjust(2);
        end
    end
    
end

