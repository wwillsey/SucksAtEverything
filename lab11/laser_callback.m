function laser_callback( src, msg )

global system laser_counter use_localization
% if(system.terminated && ~ran)
%make the raw data into processable data
if mod(laser_counter, 1) == 0
tic;
lidar_read = msg.Ranges;
rImage_dense = rangeImage(lidar_read, 1, true);
rImage_sparse = rangeImage(lidar_read, 10, true);
x = rImage_sparse.xArray;
y = rImage_sparse.yArray;
all_th = rImage_sparse.tArray;
pts = [x;y;ones(1, rImage_sparse.numPix)];

T_rs = [1,0,-10/100;0,1,0;0,0,1];

%% Updating the robot pose
if(use_localization)
    current_pose_fus = system.estRobot.robot_pose_fus;
    [succ, p_lid] = system.map.refinePose(pose(current_pose_fus), pts, 200);
    if succ
        p_lid = p_lid.getPoseVec;
        updated_pose_fus = current_pose_fus + (0.25)*(robotModel.pose_addition(p_lid, current_pose_fus, -1));
        system.estRobot.robot_pose_fus = updated_pose_fus;
    end
end
%% Filter out wall points
if(system.terminated && system.count == 0)
    x = rImage_dense.xArray;
    y = rImage_dense.yArray;
    all_th = rImage_dense.tArray;
    pts = [x;y;ones(1, rImage_dense.numPix)];
    nonwallpts = [];
    nonwallth = [];
    for i = 1:size(pts, 2)
        po = pts(:, i);
        robot_pts = T_rs * po;
        abspose = system.relToAbs(robot_pts);
        if(abs(abspose(1)) >= 0.1 && abs(abspose(2)) >= 0.1)
            nonwallpts = [nonwallpts, po];
            nonwallth = [nonwallth, all_th(i)];
        end
    end
    nonwallx = nonwallpts(1, :);
    nonwally = nonwallpts(2, :);
    nonwallnum = size(nonwallpts, 2);
    %% Find the line
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
        fx = T_final(1,3)
        fy = T_final(2,3)
        if(abs(fx) >= 0.05 || abs(fy) >= 0.05)           
            robot_trajectory = cubicSpiral.planTrajectory(T_final(1,3), T_final(2,3), th, 1);
            robot_trajectory.planVelocities(0.2);
            system.trajectoryFollower.loadTrajectory(robot_trajectory, system.estRobot.robot_pose_fus);
            system.terminated = false;
        end
    end
end
laser_counter = laser_counter + 1;
end

