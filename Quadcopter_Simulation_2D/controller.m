function [ u1, u2 ] = controller(~, state, des_state, params)
% Proportional and Derivative Gains
Kp_z = 80;
Kd_z = 20;

Kp_y =20;
Kd_y = 5;

Kp_a = 900;
Kd_a = 20;

u1 = params.mass*(params.gravity + des_state.acc(2,1) + Kd_z*(des_state.vel(2,1)-state.vel(2,1)) + Kp_z*(des_state.pos(2,1)-state.pos(2,1)));

phi_c = -(1/params.gravity)*(des_state.acc(1,1) + Kd_y*(des_state.vel(1,1)-state.vel(1,1) + Kp_y*(des_state.pos(1,1)-state.pos(1,1))));
phi_c_dot = 0;
phi_c_2dot = 0;

u2 = params.Ixx*(phi_c_2dot + Kd_a*(phi_c_dot-state.omega) + Kp_a*(phi_c-state.rot));
end

