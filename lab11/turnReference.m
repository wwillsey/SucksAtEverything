classdef turnReference
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        amax;
        vmax;
        wmax;
        rad;
        t_ramp;
        poseArray;
        timeArray;
        tf;
    end
    
    methods
        function obj = turnReference(vmax, rad, initPose)
           obj.vmax = vmax;
           [V, w] = robotModel.vlvrToVw(vmax, -vmax);
           obj.wmax  = abs(w);
           obj.rad = rad;
           obj.tf = abs(rad) / abs(obj.wmax);   
           obj.timeArray = zeros(1, floor(obj.tf/0.005));
           obj.poseArray = zeros(3, floor(obj.tf/0.005));
           % integrate
           
            x = initPose(1);
            y = initPose(2);
            theta = initPose(3);
            dt = 0.005;
            index = 1;
            for t = 0:dt:obj.tf
                [V, w] = obj.computeControl(t);
                theta = theta + w*dt;
                dx = cos(theta) * V*dt;
                dy = sin(theta) * V*dt;
                x = x+dx;
                y = y+dy;
                obj.poseArray(:, index) = [x; y; theta] ;
                obj.timeArray(index) = t;
                index = index + 1;
            end
        end
        
        
        function [V, w] = computeControl(obj, timenow)
            t = timenow;
            uref = obj.vmax;
            uref = sign(obj.rad) * uref;
            [V, w] = robotModel.vlvrToVw(-uref, uref); 
        end
        
        function v = getVAtTime(obj, t)
            [v, w] = obj.computeControl(t);
        end
        
        function w = getwAtTime(obj, t)
            [v, w] = obj.computeControl(t);
        end
        
        function pose  = getPoseAtTime(obj,t)
            if (t > obj.timeArray(end))
                pose = obj.poseArray(:,end);
            else
                x = interp1(obj.timeArray,obj.poseArray(1,:),t,'pchip','extrap');
                y = interp1(obj.timeArray,obj.poseArray(2,:),t,'pchip','extrap');
                th = interp1(obj.timeArray,obj.poseArray(3,:),t,'pchip','extrap');
                pose  = [x ; y ; th];
            end
        end  
        
        function duration = getTrajectoryDuration(obj)
            duration = obj.tf;
        end
    end
    
end