classdef lineMapLocalizer < handle
 %mapLocalizer A class to match a range scan against a map in
 % order to find the true location of the range scan relative to
 % the map.

 properties(Constant)
    maxErr = 0.10; % 5 cm
    minPts = 5; % min # of points that must match
 end

 properties(Access = private)
 end

 properties(Access = public)
    lines_p1 = [];
    lines_p2 = [];
    gain = 0.0;
    errThresh = 0.0;
    gradThresh = 0.0;
 end

methods
    function obj = lineMapLocalizer(lines_p1,lines_p2,gain,errThresh,gradThresh)
    % create a lineMapLocalizer
        obj.lines_p1 = lines_p1;
        obj.lines_p2 = lines_p2;
        obj.gain = gain;
        obj.errThresh = errThresh;
        obj.gradThresh = gradThresh;
    end
    function ro2 = closestSquaredDistanceToLines(obj,pi)
    % Find the squared shortest distance from pi to any line
    % segment in the supplied list of line segments.
    % pi is an array of 2d points
    % throw away homogenous flag
        pi = pi(1:2,:);
        r2Array = zeros(size(obj.lines_p1,2),size(pi,2));
        for i = 1:size(obj.lines_p1,2)
            [r2Array(i,:) , ~] = closestPointOnLineSegment(pi,...
            obj.lines_p1(:,i),obj.lines_p2(:,i));
        end
        ro2 = min(r2Array,[],1);    
    end
    
    function ids = throwOutliers(obj,pose,ptsInModelFrame)
        % Find ids of outliers in a scan.
        worldPts = pose.bToA()*ptsInModelFrame;
        r2 = obj.closestSquaredDistanceToLines(worldPts);
        ids = find(sqrt(r2) > obj.maxErr);
    end
    
    function avgErr = fitError(obj,pose,ptsInModelFrame)
    % Find the standard deviation of perpendicular distances of
    % all points to all lines
    % transform the points
        worldPts = pose.bToA()*ptsInModelFrame;

        r2 = obj.closestSquaredDistanceToLines(worldPts);
        r2(r2 == Inf) = [];
        err = sum(r2);
        num = length(r2);
        if(num >= lineMapLocalizer.minPts)
            avgErr = sqrt(err)/num;
        else
        % not enough points to make a guess
            avgErr = inf;
        end
    end
    
    function [errPlus0,J] = getJacobian(obj,poseIn,modelPts)
        % Computes the gradient of the error function

        errPlus0 = fitError(obj,poseIn,modelPts);

        eps = 0.001;
        dp = [eps ; 0.0 ; 0.0];
        newPose = pose(poseIn.getPoseVec+dp);
        dx = (fitError(obj, newPose, modelPts) - errPlus0)/eps;
        dp = [0.0; eps; 0.0];
        newPose = pose(poseIn.getPoseVec+dp);
        dy = (fitError(obj, newPose, modelPts) - errPlus0)/eps;
        dp = [0.0; 0.0; eps];
        newPose = pose(poseIn.getPoseVec+dp);
        dth = (fitError(obj, newPose, modelPts) - errPlus0)/eps;
        J = [dx dy dth];
    end
    
    function [success, outPose] = refinePose(obj,thePose,ptsInModelFrame,maxIters)
        err = 1000;
        J = [10,10,10];
        count = 0;
        thePose = pose(robotModel.senToWorld(thePose));
        ids = obj.throwOutliers(thePose, ptsInModelFrame);
        ptsInModelFrame(:,ids) = [];
        modelPts = ptsInModelFrame;
        while err > 0.0001 && norm(J) > 0.001 && count < maxIters
            count = count + 1;
            [err,J] = obj.getJacobian(thePose,modelPts);
            if err < .001
                thePose = pose(thePose.getPoseVec - .01*J');
            else
                thePose = pose(thePose.getPoseVec - .05*J');
            end
        end
        success = (count < maxIters);
        outPose = pose(robotModel.robToWorld(thePose));
    end
end 
end
 
