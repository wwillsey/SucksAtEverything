%debugger and global parameter initialization
global t_accum tp running;
global error_plot feed_back_plot;
tp = -1;
t_accum = 0;
figure(2)
f2 = plot(0,0,0,0);
figure(3)
f3 = plot(0,0,0,0);
feed_back_plot = plotter(f2);
error_plot = plotter(f3);