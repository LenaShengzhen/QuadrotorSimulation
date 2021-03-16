function [ desired_state ] = trajectory_generator(time, qn, path, t_time, ts_par, x_par)
% TRAJECTORY_GENERATOR: 
%
% NOTE: This function would be called with variable number of input
% arguments. In init_script, it will be called with arguments
% trajectory_generator([], [], map, path) and later, in test_trajectory,
% it will be called with only t and qn as arguments, so your code should
% be able to handle that. This can be done by checking the number of
% arguments to the function using the "nargin" variable, check the
% MATLAB documentation for more information.
%
% path: This is the path returned by your planner 
%
% desired_state: Contains all the information that is passed to the
% controller, as in phase 2
%
% It is suggested to use "persistent" variables to store map and path
% during the initialization call of trajectory_generator, e.g.
% persistent  path0
% path0 = path;
persistent path0 total_time X ts;
if numel(time) == 0 | numel(qn) == 0
    path0 = path;
    total_time = t_time;
    ts = ts_par;
    X = x_par;
    return
end

% The number of function parameters
if nargin < 3
    path = path0;
end

p = path;
if time >= total_time
    pos = p(end,:);
    vel = [0;0;0];
    acc = [0;0;0];
else
    
%     3rd order trajectory planning
%     k = find(ts<=t);
%     k = k(end);
%     pos = [1, t, t^2, t^3]*X(4*(k-1)+1:4*k,:);
%     vel = [0, 1, 2*t, 3*t^2]*X(4*(k-1)+1:4*k,:);
%     acc = [0, 0, 2,   6*t]*X(4*(k-1)+1:4*k,:);

    % 7th order minimum snap trajectory
    t = time;
    k = find(ts<=t);
    k = k(end);
    pos = [1, t, t^2, t^3, t^4, t^5, t^6, t^7]*X(8*(k-1)+1:8*k,:);
    vel = [0, 1, 2*t, 3*t^2, 4*t^3, 5*t^4, 6*t^5, 7*t^6]*X(8*(k-1)+1:8*k,:);
    acc = [0, 0, 2, 6*t, 12*t^2, 20*t^3, 30*t^4, 42*t^5]*X(8*(k-1)+1:8*k,:);
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

