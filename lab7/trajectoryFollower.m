classdef trajectoryFollower < handle
    %Wrapper class for taking a reference trajectory and feedback
    %trajectory. Computer the final velocity sent to robo based on time
    
    properties
        controller;
        robot_trajectory;
        finished;
        startPose;
        T;
    end
    
    methods
        function obj = trajectoryFollower()
            obj.controller = controller();
            obj.finished = true;
            obj.T = eye(3);
        end
        
        function loadTrajectory(obj, trajectory, startPose)
            obj.robot_trajectory = trajectory;
            obj.finished = false;
            obj.startPose = pose(startPose(1), startPose(2), startPose(3));
            obj.T = obj.T * obj.startPose.bToA();
        end
        
        function [vl, vr] = feedfoward_velocity(obj, time)
            V = obj.robot_trajectory.getVAtTime(time);
            w = obj.robot_trajectory.getwAtTime(time);
            [vl, vr] = robotModel.VwTovlvr(V, w);
        end
        
        function [vl, vr] = feedback_velocity(obj, estRobot, time)
            time_delay = 0;
            raw_refPose = obj.robot_trajectory.getPoseAtTime(time-time_delay);
            raw_refPose(3) = 1;
            refPose = obj.T * raw_refPose;
            refPose(1) = refPose(1) / refPose(3);
            refPose(2) = refPose(2) / refPose(3);
            estPose = estRobot.robot_pose;
            [V, w] = obj.controller.linear_feedback(refPose, estPose);
            [vl, vr] = robotModel.VwTovlvr(V, w);
        end
        
        function [vl, vr] = getVelocity(obj, estRobot, time, use_feedback)
            if(time >= obj.robot_trajectory.timeArray(end)+1)
                obj.finished = true;
                vl = 0;
                vr = 0;
            else
                
                if(use_feedback)
                    [vl_feedback, vr_feedback] = obj.feedback_velocity(estRobot, time);
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
    
end

