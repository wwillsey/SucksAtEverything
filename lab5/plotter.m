classdef plotter < handle
    %PLOTTER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        all_x;
        all_y;
        all_x1;
        all_y1;
        h;
    end
    
    methods
        function obj = plotter(num)
            obj.all_x = [];
            obj.all_y = [];
            obj.all_x1 = [];
            obj.all_y1 = [];
            obj.h = num;
        end
        function update_plot(obj, x, y, x1, y1)
            obj.all_x = [obj.all_x, x];
            obj.all_y = [obj.all_y, y];
            obj.all_x1 = [obj.all_x1, x1];
            obj.all_y1 = [obj.all_y1, y1];
            figure(obj.h);
            hold on;
            plot(obj.all_x, obj.all_y,'r-', obj.all_x1, obj.all_y1, 'b-');
        end 
     
    end
    
end

