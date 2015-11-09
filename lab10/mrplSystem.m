classdef mrplSystem < handle
    %MRPLSYSTEM Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        robot
        estRobot
        refRobot
        map
        trajectoryFollower
        t_accum
        t_traj
        use_feedback
        terminated
        count
    end
    
    methods
        function obj = mrplSystem(robot, lines_p1, lines_p2, init_pose, use_feedback)
            obj.robot = robot;
            obj.estRobot = sRobot(init_pose);
            obj.refRobot = [];
            obj.map = lineMapLocalizer(lines_p1, lines_p2, .01, .001, .0005);
            obj.trajectoryFollower = trajectoryFollower();
            obj.use_feedback = use_feedback;
            obj.t_accum = 0;
            obj.t_traj = 0;
            obj.terminated = false;
            obj.count = 0;
        end
        function update(obj, el_n, er_n, dt)
            obj.estRobot.update_pose(el_n, er_n, dt);
            obj.t_accum = obj.t_accum + dt;
            obj.t_traj = obj.t_traj + dt;
            if obj.trajectoryFollower.finished == true && obj.count >= 2
                obj.terminated = true;
            end
            
        end
        function executeTrajectory(obj)
            [vl, vr] = obj.trajectoryFollower.getVelocity(obj.estRobot, obj.t_traj, obj.use_feedback);
            obj.robot.sendVelocity(vl, vr);
        end
    end
end

