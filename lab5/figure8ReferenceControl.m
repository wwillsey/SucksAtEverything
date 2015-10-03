classdef figure8ReferenceControl
    %FIGURE8REFERENCECONTROL Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        KS
        KV
        pause_time 

    end
    
    methods
        function obj = figure8ReferenceControl(Ks, Kv, tPause)
            obj.KS = Ks;
            obj.KV = Kv;
            obj.pause_time = tPause;
        end
        function [V, w] = computeControl(obj, timeNow)
            t = timeNow - obj.pause_time;
            if(t < 0 || t > getTrajectoryDuration(obj))
                V = 0;
                w = 0;
            else
                vr = 0.3*obj.KV + 0.14125*obj.KV/obj.KS*sin((t*obj.KV) / (2*obj.KS));
                vl = 0.3*obj.KV - 0.14125*obj.KV/obj.KS*sin((t*obj.KV) / (2*obj.KS));
                [V, w] = robotModel.vlvrToVw(vl, vr); 
            end
        end
        function duration = getTrajectoryDuration(obj)
            duration = 12.565*obj.KS / obj.KV;
        end
    end
    
end

