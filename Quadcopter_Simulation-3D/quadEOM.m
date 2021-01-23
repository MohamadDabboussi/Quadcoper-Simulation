function sdot = quadEOM(t, s, qn, controlhandle, trajhandle, params,Data,speed,opt,dimension)
% convert state to quad stuct for control
    qd{qn} = stateToQd(s);
    if dimension==3

        % Get desired_state
        desired_state = trajhandle(t, qn,Data,speed,opt,3);

        % The desired_state is set in the trajectory generator
        qd{qn}.pos_des      = desired_state.pos;
        qd{qn}.vel_des      = desired_state.vel;
        qd{qn}.acc_des      = desired_state.acc;
        qd{qn}.yaw_des      = desired_state.yaw;
        qd{qn}.yawdot_des   = desired_state.yawdot;

    elseif dimension == 2
        if ~exist('speed','var')
          speed = 1;
         end
         if ~exist('opt','var')
              speed = 7;
         end


        % Get desired_state

        if (isequal(trajhandle,@KeysTraj))
        desired_state = trajhandle(t, qn);
        % The desired_state is set in the trajectory generator
        qd{qn}.pos_des      = qd{qn}.pos;
        qd{qn}.vel_des      = Data';
        qd{qn}.acc_des      = desired_state.acc;
        qd{qn}.yaw_des      = 0;
        qd{qn}.yawdot_des   = 0;
        end

        if (isequal(trajhandle,@trajectory_generator))
            desired_state = trajhandle(t, qn,Data,speed,opt,2);

            % The desired_state is set in the trajectory generator
            qd{qn}.pos_des      = desired_state.pos;
            qd{qn}.vel_des      = desired_state.vel;
            qd{qn}.acc_des      = desired_state.acc;
            qd{qn}.yaw_des      = desired_state.yaw;
            qd{qn}.yawdot_des   = desired_state.yawdot;
        end



    end

    % get control outputs
    [F, M, trpy, drpy] = controlhandle(qd, t, qn, params,dimension);

    % compute derivative
    sdot = quadEOM_readonly(t, s, F, M, params,dimension,qn);
    
end
