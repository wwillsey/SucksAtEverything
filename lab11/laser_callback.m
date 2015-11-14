function laser_callback( src, msg )

global system laser_counter
% if(system.terminated && ~ran)
%make the raw data into processable data

if mod(laser_counter, 3) == 0
tic;
lidar_read = msg.Ranges;
rImage = rangeImage(lidar_read, 15, true);
x = rImage.xArray;
y = rImage.yArray;
pts = [x;y;ones(1, rImage.numPix)];

%Updating the robot pose
current_pose_fus = system.estRobot.robot_pose_fus;
[succ, p_lid] = system.map.refinePose(pose(current_pose_fus), pts, 200);
if succ
    p_lid = p_lid.getPoseVec;
    updated_pose_fus = current_pose_fus + (0.25)*(robotModel.pose_addition(p_lid, current_pose_fus, -1));
    system.estRobot.robot_pose_fus = updated_pose_fus;
end

%find the line 
 [pts, th, best] = rImage.find_all_lines(0.0001, 4);
 err = 0.0001;
 if(size(pts) == 0)
    disp('NO LINE FOUND');
 else
    x = rImage.xArray(best(1));
    y = rImage.yArray(best(1));
    th = best(2)
    %     th = 0;
    T_og = [1,0,-19.5/100; 0,1,0;0,0,1];
    T_so = [cos(th), -sin(th), x; sin(th), cos(th), y; 0,0,1];
    T_rs = [1,0,-10/100;0,1,0;0,0,1];
    T_final = T_rs*T_so*T_og;
    robot_trajectory = cubicSpiral.planTrajectory(T_final(1,3), T_final(2,3), th, 1);
    robot_trajectory.planVelocities(0.2);
    system.trajectoryFollower.loadTrajectory(robot_trajectory, [0,0,0]);
    system.terminated = false;
 end

laser_counter = laser_counter + 1;
end

