function [desired_state] = KeysTraj(t, qn,Data,s,o)
    desired_state.pos =[1;0;0];
    desired_state.vel =[0.1;0;0];
    desired_state.acc =[0;0;0];
    desired_state.yaw =0;
    desired_state.yawdot = 0;
end