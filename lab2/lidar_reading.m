function nearest_obj =  lidar_reading(msg)
% given a robot that has the lidar started, it reads the lidar reading and
% plot the data.
maxObjectRange = 1.5;
minObjectRange = 0.006;
maxBearing = pi/2;
lidar_read = msg.Ranges;
nearest_range = [0,2];
if(ishandle(1))
    clf(1);
else
    figure(1);
end
<<<<<<< HEAD
axis([-2, 2, -2, 2]);
nearest_obj = [1,0,0];
=======
axis([-1, 1, 0, 2]);
res = [1,0,0];
>>>>>>> 49c677aaefb08efa9c28dd0cc49a20d44964dbbd
for j = 1:360
   r = lidar_read(j);
   [x,y,b] = irToXy(j, r);
   if(r > minObjectRange & r < maxObjectRange & abs(b) < maxBearing) 
    hold on;
<<<<<<< HEAD
    plot(x,y, '*');
    if(r < nearest_range(2))
        nearest_range = [j,r];
    end
=======
    plot(-y,x, '*');
    res = [x,y,b];
>>>>>>> 49c677aaefb08efa9c28dd0cc49a20d44964dbbd
   end
end
pause(0.005);
[tx,ty,tb]=irToXy(nearest_range(1), nearest_range(2));
nearest_obj = [tx,ty,tb];
