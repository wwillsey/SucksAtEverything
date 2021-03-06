classdef trapezoidaStepReferenceControl
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        amax;
        vmax;
        dist;
        t_ramp;
        tf;
    end
    
    methods
        function obj = trapezoidaStepReferenceControl(amax, vmax, dist)
           obj.amax = amax;
           obj.vmax = vmax;
           obj.dist = dist;
           obj.t_ramp = vmax / amax;
           obj.tf = (abs(dist) + vmax^2 / amax) / vmax;    
        end
        function [V, w] = computeControl(obj, timenow)
            t = timenow;
            uref = 0;
            if t < obj.t_ramp && t > 0
                uref = obj.amax*t;
            elseif (obj.tf-t) < obj.t_ramp && obj.tf-t > 0 && t > 0
                uref = obj.amax * (obj.tf - t);
            elseif obj.t_ramp < t && t < obj.tf - obj.t_ramp && t > 0
                uref = obj.vmax;
            else
                uref = 0;
            end
            uref = sign(obj.dist) * uref;
            [V, w] = robotModel.vlvrToVw(uref, uref); 
        end
        
        function duration = getTrajectoryDuration(obj)
            duration = obj.tf;
        end
    end
    
end

