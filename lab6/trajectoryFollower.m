classdef trajectoryFollower < handle
    %Wrapper class for taking a reference trajectory and feedback
    %trajectory. Computer the final velocity sent to robo based on time
    
    properties
        controller;
        robot_trajectory;
        finished;
    end
    
    methods
        function obj = trajectoryFollower(robot_trajectory, controller)
            obj.robot_trajectory = robot_trajectory;
            obj.controller = controller;
            obj.finished = false;
        end
        function [vl, vr] = feedfoward_velocity(obj, time)
            V = obj.robot_trajectory.getVAtTime(time);
            w = obj.robot_trajectory.getwAtTime(time);
            [vl, vr] = robotModel.VwTovlvr(V, w);
        end
        function [vl, vr] = feedback_velocity(obj, time)
            [V, w] = obj.controller.feedback(obj.robot_trajectory, time);
            [vl, vr] = robotModel.VwTovlvr(V, w);
        end
        function [vl, vr] = getVelocity(obj, time, use_feedback)
            if(time >= obj.robot_trajectory.timeArray(end)+1)
                obj.finished = true;
            end
            if(use_feedback)
                [vl_feedback, vr_feedback] = obj.feedback_velocity(time);
            end
            [vl_forward, vr_forward] = obj.feedfoward_velocity(time);
            
            if(use_feedback)
                vl = vl_forward + vl_feedback;
                vr = vr_forward + vr_feedback;
            else
                vl = vl_forward;
                vr = vr_forward;
            end
        end
        
    end
    
end

