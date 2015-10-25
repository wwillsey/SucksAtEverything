classdef rangeImage < handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Constant)
        maxUsefulRange = 2.0;
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
            while(counter < size(obj.rArray, 2))
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
        
        function [midpt, th, best] = find_all_lines(obj, err, num)
            i = 1;
            finished = false;
            midpt = [];
            th = [];
            best = [i 0];
            best_err = 0;
            while(~finished)
                [erre, nume, the] = obj.findLineCandiate(i, 12);
                if erre < err && nume >= num
                    midpt = [midpt, i];
                    th = [th the];
                    if nume > best_err
                        best_err = nume;
                        best = [i, the];
                    end
                end
                i = obj.inc(i);
                
                if(i == 1)
                    finished = true;
                end
            end
        end
        
        function [err, num, th] = findLineCandiate(obj, middle, maxLen)
            left_end = middle;
            right_end = middle;
            counter = 1;
            valid = true;
            start_range = obj.rArray(middle);
            err = 0;
            num = 0;
            th = 0;
            dist = 0;
            while(dist < maxLen && counter < obj.numPix -3 && valid)
                left_end = dec(obj, left_end);
                right_end = inc(obj, right_end);
                counter = counter + 2;
                if abs(obj.rArray(left_end) - start_range) > 0.1
                    left_end = obj.inc(left_end);
                    right_end = obj.dec(right_end);
                    break;
                elseif abs(obj.rArray(right_end) - start_range) > 0.1
                    left_end = obj.inc(left_end);
                    right_end = obj.dec(right_end);
                    break;
                else
                    x_left = obj.xArray(left_end);
                    y_left = obj.yArray(left_end);
                    x_right = obj.xArray(right_end);
                    y_right = obj.yArray(right_end);
                    dist = sqrt((x_left - x_right)^2 + (y_left-y_right)^2)*100;
                end
            end
            while(obj.rArray(left_end) == 0)
                left_end = inc(obj, left_end);
            end
            while(obj.rArray(right_end) == 0)
                right_end = dec(obj, right_end);
            end
            
            x_left = obj.xArray(left_end);
            y_left = obj.yArray(left_end);
            x_right = obj.xArray(right_end);
            y_right = obj.yArray(right_end);
            
%             x_mid = (x_left + x_right) / 2;
%             y_mid = (y_left + y_right) / 2;

            th = atan2(x_left - x_right,-y_left + y_right);
            dist = sqrt((x_left - x_right)^2 + (y_left-y_right)^2)*100;
            if(dist < 5 || dist > 15)
                err = 10000;
            else
                m = (y_right - y_left)/ (x_right - x_left);
                x_c = left_end;
                while(x_c ~= right_end)
                    if(obj.rArray(x_c) ~= 0)
                        current_x = obj.xArray(x_c);
                        current_y = obj.yArray(x_c);
                        y_diff = m*(current_x - x_left) + y_left;
                        err = err + (y_diff - current_y)^2;
                        num = num+1;
                    end
                    x_c = inc(obj, x_c);
                end
                if num == 0
                    err = 10000;
                else
                    err = err / num;
                end
            end
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

