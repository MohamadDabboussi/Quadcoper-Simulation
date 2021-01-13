function [ sdot ] = sys_eom(t, s, controlhandle, trajhandle, params)
% sys_eom Differential equation for the 2D quadrtotor system
%   [y; z; phi; y_dot; z_dot; phi_dot]

current_state.pos = s(1:2);
current_state.rot = s(3);
current_state.vel = s(4:5);
current_state.omega = s(6);

desired_state = trajhandle(t, current_state);
[F, M] = controlhandle(t, current_state, desired_state, params);

u1 = 0.5*(F - M/params.arm_length);
u2 = 0.5*(F + M/params.arm_length);

u1_clamped = min(max(params.minF/2, u1), params.maxF/2);
u2_clamped = min(max(params.minF/2, u2), params.maxF/2);
F_clamped = u1_clamped + u2_clamped;
M_clamped = (u2_clamped - u1_clamped)*params.arm_length;

sdot = [s(4);
        s(5);
        s(6);
        -F_clamped*sin(s(3))/params.mass;
        F_clamped*cos(s(3))/params.mass - params.gravity;
        M_clamped/params.Ixx];

end
