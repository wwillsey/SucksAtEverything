t = 0;
k = 2;
dt = 0.001;
t_f = 11.207*k;
x = 0;
y = 0;
theta = 0;
figure(1);
axis([-2 2 -2 2]);
fx = zeros(ceil((t_f-t)/dt));
fy = zeros(ceil((t_f-t)/dt));
index = 2;
while(t < t_f)
    vr = 1000*(0.1/k + 0.01174*t/k^2);
    vl = 1000*(0.1/k - 0.01174*t/k^2);
    w = (vr - vl) / 0.2;
    V = (vr + vl) / 2;
    theta = theta + w*dt;
    x = x + V*cos(theta)*dt;
    y = y + V*sin(theta)*dt;
    fx(index) = x;
    fy(index) = y;
    index = index+1;
    t = t + dt;
end

plot(fx, fy);