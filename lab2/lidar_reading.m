function  lidar_reading( robot )
% given a robot that has the lidar started, it reads the lidar reading and
% plot the data.
maxObjectRange = 1.5;
maxBearing = pi/2;
lidar_read = robot.laser.LatestMessage.Ranges;
figure;
for j = 1:360
   r = lidar_read(j);
   [x,y,b] = irToXy(j, r);
   if(r > 0.006 & r < maxObjectRange & abs(b) < maxBearing) 
    hold on;
    plot(x,y, '*');
   end
end
