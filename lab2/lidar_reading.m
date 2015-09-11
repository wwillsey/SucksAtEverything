function res =  lidar_reading(msg)
% given a robot that has the lidar started, it reads the lidar reading and
% plot the data.
maxObjectRange = 1.5;
maxBearing = pi/2;
lidar_read = msg.Ranges;
if(ishandle(1))
    clf(1);
end
axis([-1, 1, 0, 2]);
res = [1,0,0];
for j = 1:360
   r = lidar_read(j);
   [x,y,b] = irToXy(j, r);
   if(r > 0.006 & r < maxObjectRange & abs(b) < maxBearing) 
    hold on;
    plot(-y,x, '*');
    res = [x,y,b];
   end
   
end
