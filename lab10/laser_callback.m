function laser_callback( src, msg )

global system ran map
% if(system.terminated && ~ran)
%make the raw data into processable data
lidar_read = msg.Ranges;
rImage = rangeImage(lidar_read, 10, true);
x = rImage.xArray;
y = rImage.yArray;
pts = [x;y;ones(1, rImage.numPix)];

%Updating the robot pose
current_pose_fus = system.estRobot.robot_pose_fus;
[succ, p_lid] = system.map.refinePose(pose(current_pose_fus), pts, 1000);
p_lid = p_lid.getPoseVec;
updated_pose_fus = robotModel.pose_addition(current_pose_fus, (0.2)*(robotModel.pose_addition(p_lid, current_pose_fus, -1)), 1);
system.estRobot.robot_pose_fus = updated_pose_fus;

end

