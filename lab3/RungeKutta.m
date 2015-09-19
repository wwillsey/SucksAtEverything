function y_d = RungeKutta(f, y_n, h)
%4th order Runge-Kutta algorithm for intergration.
k1 = f(t, y_n);
k2 = f(t + h/2, y+h/2*k1);
k3 = f(t+h/2, y+k/2*k2);
k4 = f(t+h, y*k3);

y_d = y_n + h/6*(k1+2*k2+2*k3+k4);

end

