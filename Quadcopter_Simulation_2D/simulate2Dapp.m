function [t_out, s_out] = simulation_2dapp(controlhandle, trajhandle,MainPlot,splot1,splot2,splot3)

params = sys_params;

% real-time
real_time = true;

%% **************************** FIGURES *****************************
disp('Initializing figures...')
quadcolors = lines(1);
set(gcf,'Renderer','OpenGL')

%% *********************** INITIAL CONDITIONS ***********************
t_total  = 7;             % Total simulated time
tstep    = 0.01;          % this determines the time step at which the solution is given
cstep    = 0.05;          % image capture time interval
max_iter = t_total/cstep; % max iteration
nstep    = cstep/tstep;
time     = 0; % current time
err = []; % runtime errors
% Get start and stop position
des_start = trajhandle(0,[]);
des_stop  = trajhandle(inf,[]);

% Get boundary
d_state = nan(max_iter,2);
for iter = 1:max_iter
    dd = trajhandle(cstep*iter,[]);
    d_state(iter,:) = dd.pos(1:2)';
end
y_lim = [min(d_state(:,1)) - 0.1, max(d_state(:,1)) + 0.1];
z_lim = [min(d_state(:,2)) - 0.1, max(d_state(:,2)) + 0.1];
if(4*(z_lim(2) - z_lim(1)) < y_lim(2) - y_lim(1))
    z_lim(1) = z_lim(1) - (y_lim(2) - y_lim(1))/8;
    z_lim(2) = z_lim(2) + (y_lim(2) - y_lim(1))/8;
end
stop_pos = des_stop.pos;
x0        = [des_start.pos; 0; des_start.vel; 0];
xtraj     = nan(max_iter*nstep, length(x0));
ttraj     = nan(max_iter*nstep, 1);

x         = x0;        % state

pos_tol = 0.01;
vel_tol = 0.02;
ang_tol = 0.05;

%% ************************* RUN SIMULATION *************************
disp('Simulation Running....')
% Main loop
for iter = 1:max_iter

  timeint = time:tstep:time+cstep;

  tic;
  % Initialize quad plot
  if iter == 1
    MainPlot
    quad_state = simStateToQuadState(x0);
    QP = QuadPlot(1, quad_state, params.arm_length, 0.05, quadcolors(1,:), max_iter, MainPlot);
    %MainPlot.YLim(y_lim); MainPlot.ZLim(z_lim);
    quad_state = simStateToQuadState(x);
    QP.UpdateQuadPlot(quad_state, time);
    h_title = title(MainPlot, sprintf('iteration: %d, time: %4.2f', iter, time));
  end

  % Run simulation
  [tsave, xsave] = ode45(@(t,s) sys_eom(t, s, controlhandle, trajhandle, params), timeint, x);
  x = xsave(end, :)';

  % Save to traj
  xtraj((iter-1)*nstep+1:iter*nstep,:) = xsave(1:end-1,:);
  ttraj((iter-1)*nstep+1:iter*nstep) = tsave(1:end-1);

  % Update quad plot
  quad_state = simStateToQuadState(x);
  QP.UpdateQuadPlot(quad_state, time + cstep);
  MainPlot
  MainPlot.YLim(y_lim); MainPlot.ZLim(z_lim);
  set(h_title, 'String', sprintf('iteration: %d, time: %4.2f', iter, time + cstep))
  time = time + cstep; % Update simulation time
  
    splot1
    plot(splot1,ttraj(1:iter*nstep), xtraj(1:iter*nstep,1));
    %splot1.XLabel('t [s]'); splot1.YLabel('y [m]');
    grid on;
    splot2
    plot(ttraj(1:iter*nstep), xtraj(1:iter*nstep,2));
    %splot2.XLabel('t [s]'); splot2.YLabel('z [m]');
    grid on;
    splot3
    plot(ttraj(1:iter*nstep), 180/pi*xtraj(1:iter*nstep,3));
    grid on;
    %splot3.XLabel('t [s]'); splot3.YLabel('\phi [deg]');

  t = toc;
  % Check to make sure ode45 is not timing out
  if(t > cstep*50)
    err = 'Ode45 Unstable';
    break;
  end

  % Pause to make real-time
  if real_time && (t < cstep)
    pause(cstep - t);
  end

  % Check termination criteria
  if norm(x(1:2) - stop_pos) < pos_tol && norm(x(4:5)) < vel_tol && abs(x(3)) < ang_tol
    err = [];
    break
  end
  err = 'Did not reach goal';
end
disp('Simulation done');

if ~isempty(err)
  disp(['Error: ', err]);
else
  disp(['Final time: ', num2str(time), ' sec']);
end
t_out = ttraj(1:iter*nstep);
s_out = xtraj(1:iter*nstep,:);

end
