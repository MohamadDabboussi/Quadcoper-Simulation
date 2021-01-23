function params = crazyflie()


m = 0.3;  % weight (in kg) with 5 vicon markers (each is about 0.25g)
g = 9.81;   % gravitational constant
I = [1.43e-5,   0,          0; % inertial tensor in m^2 kg
     0,         1.43e-5,    0;
     0,         0,          2.89e-5];
L = 0.225; % arm length in m

params.mass = m;
params.I    = I;
params.invI = inv(I);
params.grav = g;
params.arm_length = L;

params.maxangle = 85*pi/180; % you can specify the maximum commanded angle here
params.maxF     = 2.5*m*g;   % left these untouched from the nano plus
params.minF     = 0.05*m*g;  % left these untouched from the nano plus

% You can add any fields you want in params
% for example you can add your controller gains by
% params.k = 0, and they will be passed into controller.m

end
