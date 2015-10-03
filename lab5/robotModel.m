classdef robotModel
    %UNTITLED2 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties(Constant)
        W = 0.235;
        W2 = 0.235 / 2;
    end
    
    methods(Static = true)
        function [V, w] = vlvrToVw(vl, vr)
            V = (vl+vr)/ 2;
            w = (vr - vl) / robotModel.W;
        end
        function [vl, vr] = VwTovlvr(V, w)
            vr = (2*V + w*robotModel.W) / 2;
            vl = (2*V - w*robotModel.W) / 2;
        end
    end
    
end

