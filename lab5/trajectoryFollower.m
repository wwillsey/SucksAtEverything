classdef trajectoryFollower
    %TRAJECTORY Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        feedforward;
        controller;
        reference;
    end
    
    methods
        function obj = trajectoryFollower(feedforward_control, reference_traj, controller)
            obj.feedforward = feedforward_control;
            obj.reference = reference_traj;
            obj.controller = controller;
        end
        function [vl, vr] = feedfoward_velocity(obj, time)
            [V, w] = obj.feedfoward_controller.computeControl(time);
            [vl, vr] = robotModel.VwTovlvr(V, w);
        end
        function [vl, vr] = feedback_velocity(obj, time)
            [V, w] = obj.controller.feedback(obj.reference, time);
            [vl, vr] = robotModel.VwTovlvr(V, w);
        end
        function [vl, vr] = getVelocity(obj, time, use_feedback)
            [vl_forward, vr_forward] = obj.feedfoward_velocity(time);
            [vl_feedback, vr_feedback] = obj.feedback_velocity(time);
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

