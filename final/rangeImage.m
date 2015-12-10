classdef rangeImage < handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Constant)
        maxUsefulRange = 3.0;
        minUsefulRange = 0.05;
        maxRangeForTarget = 1.0;
    end
    
    properties (Access = public)
        rArray = [];
        tArray = [];
        xArray = [];
        yArray = [];
        numPix;
    end
    
    methods (Access = public)
        function obj = rangeImage(ranges, skip, cleanFlag)
            if(nargin == 3)
                n=0;
                for i = 1:skip:length(ranges)
                    n = n+1;
                    obj.rArray(n) = ranges(i);
                    obj.tArray(n) = (i-1) * (pi/180);
                    obj.xArray(n) = ranges(i) * cos(obj.tArray(n));
                    obj.yArray(n) = ranges(i) * sin(obj.tArray(n));
                end
                obj.numPix = n;
                if cleanFlag; obj.removeBadPoints(); end;
            end
        end
        
        function removeBadPoints(obj)
            counter = 1;
            while(counter <= size(obj.rArray, 2))
                if(obj.rArray(counter) <= obj.minUsefulRange || ...
                    obj.rArray(counter) >= obj.maxUsefulRange)
                    obj.rArray(counter) = [];
                    obj.tArray(counter) = [];
                    obj.xArray(counter) = [];
                    obj.yArray(counter) = [];
                else
                    counter = counter + 1;
                end
            end
            obj.numPix = size(obj.rArray, 2);
        end
        
        function plotRvsTh(obj, maxRange)
            r_new = [];
            t_new = [];
            for i = 1:obj.numPix
                if(obj.rArray(i) <= maxRange)
                    r_new = [r_new obj.rArray(i)];
                    t_new = [t_new obj.tArray(i)];
                end
            end
            plot(r_new, t_new, '*');
        end
        function plotXvsY(obj, maxRange)
            x_new = [];
            y_new = [];
            for i = 1:obj.numPix
                if(sqrt(obj.xArray(i)^2 + obj.yArray(i)^2) <= maxRange)
                    x_new = [x_new obj.xArray(i)];
                    y_new = [y_new obj.yArray(i)];
                end
            end
            plot(-y_new, x_new, '*');
            axis([-2,2,-2,2]);
            axis square;
        end
         function plotXvsY_world(obj, robotPose, maxRange)
            x_new = [];
            y_new = [];
            count = 0;
            for i = 1:obj.numPix
                if(sqrt(obj.xArray(i)^2 + obj.yArray(i)^2) <= maxRange)
                    x_new = [x_new obj.xArray(i)];
                    y_new = [y_new obj.yArray(i)];
                    count = count+1;
                end
            end
            pts = [x_new;y_new; ones(1, count)];
            world_pts = robotModel.senToWorld(robotPose) * pts;
            x_data = world_pts(1,:);
            y_data = world_pts(2,:);
            plot(-y_data, x_data, '*');
            axis([-1,1,-1,1]);
            axis square;
            
        end
        function [x, y, th] = irToXy( i, r )
        % irToXy finds position and bearing of a range pixel endpoint
        % Finds the position and bearing of the endpoint of a range pixel
        % in the plane.
            if i <= 180
                th = degtorad(i);
            else
                th = degtorad(i - 360);
            end

            y = r * sind(i);
            x = r * cosd(i);
        end
        
        function num = numPixels(obj)
            num = obj.numPix;
        end
        
        function out = inc(obj, in)
            out = indexAdd(obj, in, 1);
        end
        
        function out = dec(obj, in)
            out = indexAdd(obj, in, -1);
        end
        
        function out = indexAdd(obj, a, b)
            out = mod((a-1) + b, obj.numPix) + 1;
        end
    end
    
end

