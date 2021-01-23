function [ terminate_cond ] = terminate_check( x, time, stop, pos_tol, vel_tol, time_tol,qn)
%TERMINATE_CHECK Check termination criteria, including position, velocity and time


% Initialize
pos_check = true;
vel_check = true;


% Check position and velocity and still time for each quad
    pos=x{qn}(1:3) - stop{qn};
    pos_check = pos_check && (norm(pos) < pos_tol);
    vel_check = vel_check && (norm(x{qn}(4:6)) < vel_tol);



% Check total simulation time
time_check = time > time_tol;
% Check collision criteria


if (pos_check && vel_check)
    terminate_cond = 1; % Robot reaches goal and stops, successful
elseif time_check
    terminate_cond = 2; % Robot doesn't reach goal within given time, not complete
else
    terminate_cond = 0;
end

end