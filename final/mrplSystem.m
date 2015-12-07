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
            obj.terminated = true;
            obj.count = 0;
        end
        function update(obj, el_n, er_n, dt)
            obj.estRobot.update_pose(el_n, er_n, dt);
            obj.t_accum = obj.t_accum + dt;
            obj.t_traj = obj.t_traj + dt;
            if obj.trajectoryFollower.finished == true && obj.count > 5
                obj.terminated = true;
            end
            
        end
        function executeTrajectory(obj)
            [vl, vr] = obj.trajectoryFollower.getVelocity(obj.estRobot, obj.t_traj, obj.use_feedback);
            obj.robot.sendVelocity(vl, vr);
        end
        
        function rel_pose = absToRel(obj, abs_pose)
            robot_pose = obj.estRobot.robot_pose_fus;
            rx = robot_pose(1);
            ry = robot_pose(2);
            rt = robot_pose(3);
            Hwr = [cos(rt), -sin(rt), rx; sin(rt), cos(rt), ry; 0 0 1];
            rel_position = (Hwr)^-1 * [abs_pose(1); abs_pose(2); 1];
            th = atan2(sin(abs_pose(3) - rt), cos(abs_pose(3) - rt));
            rel_pose = [rel_position(1), rel_position(2), th];
        end
        function abs_pose = relToAbs(obj, rel_pose)
            robot_pose = obj.estRobot.robot_pose_fus;
            rx = robot_pose(1);
            ry = robot_pose(2);
            rt = robot_pose(3);
            Hwr = [cos(rt), -sin(rt), rx; sin(rt), cos(rt), ry; 0 0 1];
            abs_position = (Hwr) * [rel_pose(1); rel_pose(2); 1];
            th = atan2(sin(rel_pose(3) - rt), cos(rel_pose(3) - rt));
            abs_pose = [abs_position(1), abs_position(2), th];
        end
        
    end
end

