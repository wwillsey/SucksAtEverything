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
end

laser_counter = laser_counter + 1;
end

