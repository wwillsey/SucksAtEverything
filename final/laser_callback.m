function laser_callback( src, msg )

global system laser_counter use_localization acquiring acquired
% if(system.terminated && ~ran)
%make the raw data into processable data
lidar_read = msg.Ranges;

if(acquiring)
    rImage_dense = rangeImage(lidar_read, 1, true);
    x = rImage_dense.xArray;
    y = rImage_dense.yArray;
    all_th = rImage_dense.tArray;
    T_rs = [1,0,-10/100;0,1,0;0,0,1];
    pts = [x;y;ones(1, rImage_dense.numPix)];
    nonwallpts = [];
    nonwallth = [];
    for i = 1:size(pts, 2)
        po = pts(:, i);
        robot_pts = T_rs * po;
        abspose = system.relToAbs(robot_pts);
        % CHANGE this cuz map is different. 
        if(abs(abspose(1)) >= 0.05 && abs(abspose(2)) >= 0.05 && abs(abspose(1)) <= util.f2m(8) - 0.1 ...
                && abs(robot_pts(2)) < .10 && robot_pts(1) > 0)
            nonwallpts = [nonwallpts, po];
            nonwallth = [nonwallth, all_th(i)];
        end
    end
    nonwallx = nonwallpts(1, :);
    nonwally = nonwallpts(2, :);
    nonwallnum = size(nonwallpts, 2);
    
    
    %% Find the line
    % do we need to change the linefinder cuz theres 3 walls now? idk if
    % there is
    pts = object_finder.find_line_ht(nonwallx, nonwally, nonwallth, nonwallnum);
    if(size(pts) == 0)
        disp('NO LINE FOUND');
    else
        x = pts(1);
        y = pts(2);
        th = pts(3);
        T_og = [1,0,-18/100; 0,1,0;0,0,1];
        T_so = [cos(th), -sin(th), x; sin(th), cos(th), y; 0,0,1];
        T_rs = [1,0,-9/100;0,1,0;0,0,1];
        T_final = T_rs*T_so*T_og;
        disp('pick up at (rel robot)');
        rImage_dense.plotXvsY(2);
        fx = T_final(1,3)
        fy = T_final(2,3)
        robot_trajectory = cubicSpiral.planTrajectory(T_final(1,3), T_final(2,3), th, 1);
        robot_trajectory.planVelocities(0.15);
        system.trajectoryFollower.loadTrajectory(robot_trajectory, system.estRobot.robot_pose_fus);
        system.t_traj = 0;
        acquiring = false;
        system.terminated = false;
    end
end

if mod(laser_counter, 3) == 0
    
rImage_sparse = rangeImage(lidar_read, 10, true);
x = rImage_sparse.xArray;
y = rImage_sparse.yArray;
all_th = rImage_sparse.tArray;
pts = [x;y;ones(1, rImage_sparse.numPix)];

    %% Updating the robot pose
    if(use_localization)
        current_pose_fus = system.estRobot.robot_pose_fus;
        try
            [succ, p_lid] = system.map.refinePose(pose(current_pose_fus), pts, 200);
         if succ
            p_lid = p_lid.getPoseVec;
            p_lid(3)
            updated_pose_fus = current_pose_fus + (0.1)*(robotModel.pose_addition(p_lid, current_pose_fus, -1));
            system.estRobot.robot_pose_fus = updated_pose_fus;
         end
         catch
            disp('error')
        end
    end

end
laser_counter = laser_counter + 1;

end

