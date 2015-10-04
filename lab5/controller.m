classdef controller < handle
    %UNTITLED2 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        robot_pose;
        robot_state;
    end
    
    methods
        function obj = controller()
        end
        function ret = init_state(obj, el, er) 
            obj.robot_pose = [0 0 0];
            obj.robot_state = struct('el', el, 'er', er, 'theta', 0);
            ret = true;
        end
        function update_pose(obj, el_n, er_n, dt)
            if el_n ~= 0 && er_n ~= 0 &&  dt~=0
            del = el_n - obj.robot_state.el;
            der = er_n - obj.robot_state.er;
            vl_n = del / dt;
            vr_n = der / dt;
            [V, w] = robotModel.vlvrToVw(vl_n, vr_n);
            theta_p = obj.robot_state.theta;
            theta_n = theta_p + w * dt;
            dx = cos(double(theta_n)) * V * dt;
            dy = sin(double(theta_n)) * V * dt;
            x_p = obj.robot_pose(1);
            y_p = obj.robot_pose(2);
            x_n = x_p + dx;
            y_n = y_p + dy;
           % obj.robot_state = struct('el', el_n, 'er', er_n, 'theta', theta_n);
            obj.robot_state.el = el_n;
            obj.robot_state.er = er_n;
            obj.robot_state.theta = theta_n;
            obj.robot_pose = [x_n, y_n, theta_n];
            end
        end
        function [V, w] = feedback(obj, traj_ref, t)
            kpx = 1;
            kpy = 1;
            
            goal_pose = traj_ref.getPoseAtTime(t);
            goal_position = goal_pose(1:2)';
            actual_position = obj.robot_pose(1:2)';
            theta = obj.robot_pose(3);
            error_world = goal_position - actual_position;
            error = [cos(theta), -sin(theta); sin(theta), cos(theta)]^-1 * error_world;
            adjust = [kpx, 0; 0, kpy] * error;
            V = adjust(1);
            w = adjust(2);
        end
    end
    
end

