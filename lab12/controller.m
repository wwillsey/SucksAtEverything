classdef controller < handle
    %UNTITLED2 Summary of this class goes here
    %   Detailed explanation goes here
   
    properties
        robot_pose;
        robot_state;
        error;
        data_log;
    end
    
    methods(Access = public)
        function obj = controller(controller_log)
            obj.data_log = controller_log;
        end  
        
        function [V, w] = linear_feedback(obj, refPose, estPose)
            %robot_trajectory act as refrobot
            kpx = 0.0;
            kpy = 0;
            kt = 0.0;
            goal_position = refPose(1:2);
            actual_position = estPose(1:2)';
            theta = estPose(3);
            etheta = atan2(sin(refPose(3) - theta), cos(refPose(3) - theta));

            error_world = goal_position - actual_position;
            error = [cos(theta), -sin(theta); sin(theta), cos(theta)]^-1 * error_world;
             if(abs(error(1)) < 0.02)
                 kpx = 0;
             end
             if(abs(error(2)) < 0.02)
                 kpy = 0;
             end
            adjust = [kpx, 0; 0, kpy] * error + [0; kt*etheta];
            formatSpec1 = 'x:%f  y:%f refX: %f refY: %f \n e1: %f e2: %f \n';
            fprintf(obj.data_log, formatSpec1, actual_position(1), actual_position(2), goal_position(1), goal_position(2), error(1), error(2));
            V = adjust(1);
            w = adjust(2);
        end
        
        function write_log(obj, goal_position, actual_position, error)
            
        end
     end
    
end

