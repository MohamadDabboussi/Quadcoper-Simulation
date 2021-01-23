function [ desired_state ] = trajectory_generator(t, qn, path,speed,opt,dimension)
persistent path0 total_time X ts path02 total_time2 X2 ts2;
if dimension == 3
    
    
    if numel(t) == 0 
       path0 = path;
       [ts, total_time] = generate_ts(path0,speed);
       if opt == 3
            X = traj_opt3(path0, total_time,ts);
       else
           if opt == 7
               X = traj_opt7(path0, total_time,ts);
           end
       end
       return
    end


    p = path;
    if t >= total_time
        pos = p(end,:);
        vel = [0;0;0];
        acc = [0;0;0];
    else
        if opt == 3    
            %3rd order trajectory planning
            k = find(ts<=t);
            k = k(end);
            pos = [t^3, t^2, t, 1]*X(4*(k-1)+1:4*k,:);
            vel = [3*t^2, 2*t, 1, 0]*X(4*(k-1)+1:4*k,:);
            acc = [6*t, 2, 0, 0]*X(4*(k-1)+1:4*k,:);
        end
        if opt ==7
            %7th order minimum snap trajectory
                k = find(ts<=t);
                k = k(end);
                pos = [t^7, t^6, t^5, t^4, t^3, t^2, t, 1]*X(8*(k-1)+1:8*k,:);
                vel = [7*t^6, 6*t^5, 5*t^4, 4*t^3, 3*t^2, 2*t, 1, 0]*X(8*(k-1)+1:8*k,:);
                acc = [42*t^5, 30*t^4, 20*t^3, 12*t^2, 6*t, 2, 0, 0]*X(8*(k-1)+1:8*k,:);
        end
    end

    yaw = 0;
    yawdot = 0;

    % =================== Your code ends here ===================
    desired_state.ten = total_time;
    desired_state.pos = pos(:);
    desired_state.vel = vel(:);
    desired_state.acc = acc(:);
    desired_state.yaw = yaw;
    desired_state.yawdot = yawdot;
elseif dimension == 2
    if (qn==1)  
    if numel(t) == 0 
       path0 = path;
       [ts, total_time] = generate_ts(path0,speed);
       if opt == 3
            X = traj_opt3(path0, total_time,ts);
       else
           if opt == 7
               X = traj_opt7(path0, total_time,ts);
           end
       end
       return
    end
    if nargin < 3
        path = path0;
    end

    p = path;
    if t >= total_time
        pos = p(end,:);
        vel = [0;0;0];
        acc = [0;0;0];
    else
        if opt == 3    
            %3rd order trajectory planning
            k = find(ts<=t);
            k = k(end);
            pos = [t^3, t^2, t, 1]*X(4*(k-1)+1:4*k,:);
            vel = [3*t^2, 2*t, 1, 0]*X(4*(k-1)+1:4*k,:);
            acc = [6*t, 2, 0, 0]*X(4*(k-1)+1:4*k,:);
        end
        if opt ==7
            %7th order minimum snap trajectory
                k = find(ts<=t);
                k = k(end);
                pos = [t^7, t^6, t^5, t^4, t^3, t^2, t, 1]*X(8*(k-1)+1:8*k,:);
                vel = [7*t^6, 6*t^5, 5*t^4, 4*t^3, 3*t^2, 2*t, 1, 0]*X(8*(k-1)+1:8*k,:);
                acc = [42*t^5, 30*t^4, 20*t^3, 12*t^2, 6*t, 2, 0, 0]*X(8*(k-1)+1:8*k,:);
        end
    end

    end
    if (qn==2)  
    if numel(t) == 0 
       path02 = path;
       [ts2, total_time2] = generate_ts(path02,speed);
       if opt == 3
            X2 = traj_opt3(path02, total_time2,ts2);
       else
           if opt == 7
               X2 = traj_opt7(path02, total_time2,ts2);
           end
       end
       return
    end
    if nargin < 3
        path = path02;
    end

    p = path;
    if t >= total_time2
        pos = p(end,:);
        vel = [0;0;0];
        acc = [0;0;0];
    else
        if opt == 3    
            %3rd order trajectory planning
            k = find(ts2<=t);
            k = k(end);
            pos = [t^3, t^2, t, 1]*X2(4*(k-1)+1:4*k,:);
            vel = [3*t^2, 2*t, 1, 0]*X2(4*(k-1)+1:4*k,:);
            acc = [6*t, 2, 0, 0]*X2(4*(k-1)+1:4*k,:);
        end
        if opt ==7
            %7th order minimum snap trajectory
                k = find(ts2<=t);
                k = k(end);
                pos = [t^7, t^6, t^5, t^4, t^3, t^2, t, 1]*X2(8*(k-1)+1:8*k,:);
                vel = [7*t^6, 6*t^5, 5*t^4, 4*t^3, 3*t^2, 2*t, 1, 0]*X2(8*(k-1)+1:8*k,:);
                acc = [42*t^5, 30*t^4, 20*t^3, 12*t^2, 6*t, 2, 0, 0]*X2(8*(k-1)+1:8*k,:);
        end
    end
    end


    yaw = 0;
    yawdot = 0;

    % =================== Your code ends here ===================

    desired_state.pos = pos(:);
    desired_state.vel = vel(:);
    desired_state.acc = acc(:);
    desired_state.yaw = yaw;
    desired_state.yawdot = yawdot;
end

