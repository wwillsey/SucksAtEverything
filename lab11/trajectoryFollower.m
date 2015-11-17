classdef trajectoryFollower < handle
    %Wrapper class for taking a reference trajectory and feedback
    %trajectory. Computer the final velocity sent to robo based on time
    
    properties
        controller;
        robot_trajectory;
        finished;
        startPose;
        T;
        init_theta;
    end
    
    methods
        function obj = trajectoryFollower()
            controller_log = fopen('control_log', 'w');
            obj.controller = controller(controller_log);
            obj.finished = true;
            obj.T = eye(3);
        end
        
        function loadTrajectory(obj, trajectory, startPose)
            obj.robot_trajectory = trajectory;
            obj.finished = false;
            obj.startPose = pose(startPose(1), startPose(2), startPose(3));
            obj.init_theta = startPose(3);
            obj.T = obj.startPose.bToA();
        end
        
        function [V, w] = feedfoward_velocity(obj, time)
            V = obj.robot_trajectory.getVAtTime(time);
            w = obj.robot_trajectory.getwAtTime(time);
        end
        
        function [V, w] = feedback_velocity(obj, estRobot, time)
            time_delay = 0.01;
            raw_refPose = obj.robot_trajectory.getPoseAtTime(time-time_delay);
            temp_th = raw_refPose(3);
            raw_refPose(3) = 1;
            refPose = obj.T * raw_refPose;
            refPose(1) = refPose(1) / refPose(3);
            refPose(2) = refPose(2) / refPose(3);
            estPose = estRobot.robot_pose_fus;
            
            th = atan2(sin(temp_th+obj.init_theta), cos(temp_th+obj.init_theta));
            refPose(3) = th;
            [V, w] = obj.controller.linear_feedback(refPose, estPose);
        end
        
        
        function [vl, vr] = getVelocity(obj, estRobot, time, use_feedback)
            if(time >= obj.robot_trajectory.timeArray(end) || obj.finished)
                obj.finished = true;
                vl = 0;
                vr = 0;
            else
                
                if(use_feedback)
                    [V_feedback, w_feedback] = obj.feedback_velocity(estRobot, time);
                end
                [V_forward, w_forward] = obj.feedfoward_velocity(time);

                if(use_feedback)
                    V = V_forward + V_feedback;
                    w = w_forward + w_feedback;
                else
                    V = V_forward;
                    w = w_forward;
                end
                [vl, vr] = robotModel.VwTovlvr(V, w);
            end
        end
        
    end
    
end

