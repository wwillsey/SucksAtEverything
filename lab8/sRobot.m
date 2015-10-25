classdef sRobot < handle
    %estRobot is the class for estimating the position of the robot. 
    properties
        robot_pose;
        robot_state;
    end
    
    methods
        function obj = sRobot()
            obj.robot_pose = [];
        end
         function update_pose(obj, el_n, er_n, dt)
            if size(obj.robot_pose, 1) == 0
                obj.robot_pose = [0,0,0];
                obj.robot_state = struct('el', el_n, 'er', er_n, 'theta', 0);
            else
                if el_n ~= 0 && er_n ~= 0 &&  dt~=0
                    del = el_n - obj.robot_state.el;
                    der = er_n - obj.robot_state.er;
                    vl_n = del / dt;
                    vr_n = der / dt;
                    [V, w] = robotModel.vlvrToVw(vl_n, vr_n);
                    theta_p = obj.robot_state.theta;
                    theta_n = theta_p + w * dt / 2;
                    dx = cos(double(theta_n)) * V * dt;
                    dy = sin(double(theta_n)) * V * dt;
                    x_p = obj.robot_pose(1);
                    y_p = obj.robot_pose(2);
                    x_n = x_p + dx;
                    y_n = y_p + dy;
                    theta_n = theta_n + w * dt / 2;
                    obj.robot_state.el = el_n;
                    obj.robot_state.er = er_n;
                    obj.robot_state.theta = theta_n;
                    obj.robot_pose = [x_n, y_n, theta_n];
                end
            end
         end
    end
    
end

