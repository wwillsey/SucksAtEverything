classdef plotter < handle
    %PLOTTER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        all_x;
        all_y;
        all_x1;
        all_y1;
        all_x2;
        all_y2;
        t;
        h;
    end
    
    methods
        function obj = plotter(num)
            obj.all_x = [];
            obj.all_y = [];
            obj.all_x1 = [];
            obj.all_y1 = [];
            obj.all_x2 = [];
            obj.all_y2 = [];
            obj.t = [];
            obj.h = num;
        end
        function update_plot(obj, x, y, x1, y1)
            obj.all_x = [obj.all_x, x];
            obj.all_y = [obj.all_y, y];
            obj.all_x1 = [obj.all_x1, x1];
            obj.all_y1 = [obj.all_y1, y1];
            %obj.h;
            set(obj.h, {'XData'}, num2cell([obj.all_x;obj.all_x1],2), {'YData'}, num2cell([obj.all_y; obj.all_y1],2));
            %drawnow;
            %plot(obj.all_x, obj.all_y,'r-', obj.all_x1, obj.all_y1, 'b-');
        end 
        function update_plot_s(obj, x,y,theta, t)
            obj.all_x = [obj.all_x x];
            obj.all_x1 = [obj.all_x1 y];
            obj.all_x2 = [obj.all_x2 theta];
            obj.t = [obj.t t];
            
            set(obj.h, {'YData'}, num2cell([obj.all_x;obj.all_x1;obj.all_x2],2), {'XData'}, num2cell([obj.t; obj.t; obj.t],2));
            %drawnow;
        end
     
    end
    
end

