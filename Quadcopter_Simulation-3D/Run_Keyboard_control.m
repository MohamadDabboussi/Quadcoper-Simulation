% NOTE: This srcipt will not run as expected unless you fill in proper
% code in trajhandle and controlhandle
% You should not modify any part of this script except for the
% visualization part

function [QP] = Run_Keyboard_control(h_3d,trajhandle,qn,Datat,Speed,opt)
Data=[0 0 0;0 0 1];
addpath('utils')
addpath('trajectories')

init_script;
% controller
controlhandle = @controller;
% real-time 
real_time = true;
% max time
time_tol = 50;%25
% parameters for simulation
params = crazyflie();

%% **************************** FIGURES *****************************
quadcolors = lines(qn);
finish=0;
%% *********************** INITIAL CONDITIONS ***********************
j=0;
while(finish==0) 
    j=j+1;
    Data=evalin('base', 'Data');


max_iter  = 10000;      % max iteration%5000
starttime = 0;         % start of simulation in seconds
tstep     = 0.01;      % this determines the time step at which the solution is given
cstep     = 0.05;      % image capture time interval
nstep     = cstep/tstep;
time      = starttime; % current time
err       = []; % runtime errors

Data((end-1):end,:)

% Get start and stop position
 des_start = trajhandle(0, qn,Data((end-1):end,:),Speed,opt);
 des_stop  = trajhandle(inf, qn,Data((end-1):end,:),Speed,opt);
 stop{qn}  = des_stop.pos;
% 
% x0{qn}    = init_state( des_start.pos, 0 );
% xtraj{qn} = zeros(max_iter*nstep, length(x0{qn}));
% ttraj{qn} = zeros(max_iter*nstep, 1);


%x         = x0;        % state

pos_tol   = 0.01;
vel_tol   = 0.01;
%  j=0;
%  while(finish==0)    
% 
% j=j+1;
%      Data=evalin('base', 'Data');

%% ************************* RUN SIMULATION *************************
% Main loop
for iter = 1:max_iter

    
%     title(h_3d,sprintf('iteration: %d, time: %4.2f', iter, time));
    tic;

    timeint = time:tstep:time+cstep;
        % Initialize quad plot
    if j == 1
        if iter==1
x0{qn}    = init_state( des_start.pos, 0 );
xtraj{qn} = zeros(max_iter*nstep, length(x0{qn}));
ttraj{qn} = zeros(max_iter*nstep, 1);
x=x0;

        QP{qn} = QuadPlot(qn, x0{qn}, 0.1, 0.04, quadcolors(qn,:), max_iter, h_3d);
        
        desired_state = trajhandle(time, qn, Data((end-1):end,:),Speed,opt);
        QP{qn}.UpdateQuadPlot(x{qn}, [desired_state.pos; desired_state.vel], time); 
        end
    end

    FV = @(t,s) quadEOM(t, s, qn, controlhandle, trajhandle, params, Data((end-1):end,:),Speed,opt);
    % Run simulation
    [tsave, xsave] = ode45(FV , timeint, x{qn});
    x{qn}    = xsave(end, :)';
        
    % Save to traj
    xtraj{qn}((iter-1)*nstep+1:iter*nstep,:) = xsave(1:end-1,:);
    ttraj{qn}((iter-1)*nstep+1:iter*nstep) = tsave(1:end-1);
    
    pause(0.1);
    % Update quad plot
    tt =time + cstep;
    desired_state = trajhandle(tt, qn,Data((end-1):end,:),Speed,opt);
    hold(h_3d,'on')
    QP{qn}.UpdateQuadPlot(x{qn}, [desired_state.pos; desired_state.vel], tt);
    hold(h_3d,'off')   
    
    
    %finish=evalin('base', 'finish');
    
    time = time + cstep; % Update simulation time
    t = toc;
    % Check to make sure ode45 is not timing out
%     if(t> cstep*50)
%         err = 'Ode45 Unstable';
%         break;
%     end

    % Pause to make real-time
    if real_time && (t < cstep)
        pause(cstep - t);
    end

    % Check termination criteria
    if terminate_check(x, time, stop, pos_tol, vel_tol, time_tol,qn)
        break
    end
end

end
fprintf('finished.\n')