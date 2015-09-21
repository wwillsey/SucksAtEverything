tic;
ks = 0.7;
kv = 0.4;
tf = 12.565*ks/kv;
t = toc;
while( t < tf)
    t = toc;
    vr = (0.3*kv + 0.14125*kv/ks*sin(t*kv/(2*ks)));
    vl = (0.3*kv - 0.14125*kv/ks*sin(t*kv/(2*ks)));
    robot.sendVelocity(vl, vr);
    pause(0.0001);
end
robot.sendVelocity(0,0);
