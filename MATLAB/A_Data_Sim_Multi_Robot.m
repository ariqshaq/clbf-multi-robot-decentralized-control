%% A_Data_Sim_Multi_Robot.m
% Simulation: 6-Robot Warehouse Navigation with LiDAR and Human Obstacles
% Output    : sim_results.mat
% Run this script first before A_Plot_Multi_Robot.m or A_Plot_Safety_Distance

clear; clc;

%% 0. User Configuration

% -- CBF-CR Toggle --
% true  -> mu_robot > mu_obstacle (priority-based safety margin active)
% false -> use LiDAR-CBF only (robots detected via LiDAR, no robot CBF term)
cbf_cr_enabled = true;

% -- Robot index whose sensor_history is saved to .mat --
saved_sensor_robot = 4;

%% 1. Warehouse Environment

warehouse.x_size = 12;  % m
warehouse.y_size = 10;  % m

%% 2. Robot Physical Parameters

robot_params.length = 0.6;   % ellipse major axis l (front-to-back), m
robot_params.width  = 0.4;   % ellipse minor axis w (side-to-side), m
robot_params.max_v  = 1.6;   % maximum linear speed v_max, m/s
robot_params.max_w  = pi;    % maximum angular speed w_max, rad/s

%% 3. Controller Parameters

ctrl_params.gamma     = 16.0;   % CLF position gain gamma
ctrl_params.gamma_phi =  9.0;   % CLF heading gain gamma_phi
ctrl_params.lambda    =  5.0;   % CBF weight lambda
ctrl_params.mu_obstacle = 0.15;    % safety margin mu_o - static obstacles & LiDAR hits, m
ctrl_params.mu_robot    = 0.35;    % safety margin mu_r (mu_r >= 2*mu_o) - robot-robot, m

%% 4. LiDAR Parameters

lidar_params.enabled         = true;
lidar_params.range           = 12.0;  % detection range, m
lidar_params.num_samples     = 800;   % total ray samples per scan (360 deg coverage)
lidar_params.min_detect_dist = 0.1;   % minimum valid hit distance, m
lidar_params.frequency       = 10;    % scan frequency, hz

%% 5. Path Prediction Parameters

path_pred.n_steps  = 5;      % forward-simulation horizon n_steps, steps
path_pred.t_sample = 0.05;   % integration step per prediction step t_sample, s
path_pred.dist_min = 1.0;    % activate only when distance to goal e_p > dist_min eps_p, m

%% 6. Dynamic Human Obstacle Parameters

% -- Human definitions [start_x, start_y, target1_y, target2_y, speed, radius] --
human_obstacles = [
    6.0, 5.0, 7.5, 2.5, 0.6, 0.15;  % Human 1
];

num_humans = size(human_obstacles, 1);

humans = struct('id', {}, 'pose', {}, 'start_pos', {}, ...
                'target1', {}, 'target2', {}, 'speed', {}, 'radius', {}, ...
                'current_target', {}, 'direction', {});

for h = 1:num_humans
    humans(h).id          = h;
    humans(h).pose        = [human_obstacles(h,1), human_obstacles(h,2)];   % [x, y]
    humans(h).start_pos   = [human_obstacles(h,1), human_obstacles(h,2)];
    humans(h).target1     = [human_obstacles(h,1), human_obstacles(h,3)];  % move up
    humans(h).target2     = [human_obstacles(h,1), human_obstacles(h,4)];  % move down
    humans(h).speed       = human_obstacles(h,5);
    humans(h).radius      = human_obstacles(h,6);
    humans(h).current_target = 1;   % start moving toward target1
    humans(h).direction      = 1;   % 1 = moving to target1, 2 = moving to target2
end

%% 7. Scenario Setup

num_robots      = 6;
lidar_wall_segs = [
    0,               0,               warehouse.x_size, 0;               % bottom wall
    warehouse.x_size, 0,               warehouse.x_size, warehouse.y_size; % right wall
    warehouse.x_size, warehouse.y_size, 0,               warehouse.y_size; % top wall
    0,               warehouse.y_size, 0,               0;                % left wall
];
lidar_circle_obs = [
    6.0, 2.0, 0.6;
    6.0, 8.0, 0.6;
];

%% 8. Tasks, Docks, and Zones (visualization & routing references)

num_tasks = 8;
tasks = [
    3.0,  0.5,  -pi/2;      % Task 1
    9.0,  0.5,  -pi/2;      % Task 2
    11.5, 3.0,  0;          % Task 3
    11.5, 7.0,  0;          % Task 4
    9.0,  9.5,  pi/2;       % Task 5
    3.0,  9.5,  pi/2;       % Task 6
    0.5,  7.0,  pi;         % Task 7
    0.5,  3.0,  pi;         % Task 8
];

docks = [
    3,     -0.25, 1.0, 0.5;    % Dock 1
    9,     -0.25, 1.0, 0.5;    % Dock 2
    12.25,  3,    0.5, 1.0;    % Dock 3
    12.25,  7,    0.5, 1.0;    % Dock 4
    9,     10.25, 1.0, 0.5;    % Dock 5
    3,     10.25, 1.0, 0.5;    % Dock 6
   -0.25,   7,    0.5, 1.0;    % Dock 7
   -0.25,   3,    0.5, 1.0;    % Dock 8
];

zones = [
    3.0,  0.5, 1.0, 1.0;    % Dock 1
    9.0,  0.5, 1.0, 1.0;    % Dock 2
    11.5, 3.0, 1.0, 1.0;    % Dock 3
    11.5, 7.0, 1.0, 1.0;    % Dock 4
    9.0,  9.5, 1.0, 1.0;    % Dock 5
    3.0,  9.5, 1.0, 1.0;    % Dock 6
    0.5,  7.0, 1.0, 1.0;    % Dock 7
    0.5,  3.0, 1.0, 1.0;    % Dock 8
];

%% 9. Robot Initialisation

% -- Parking spots [x, y, phi] --
initial_poses = [
    1.0,  9.5, -pi/2;   % Robot 1
    0.5,  5.0,  0;      % Robot 2
    1.0,  0.5,  pi/2;   % Robot 3
    11.0, 0.5,  pi/2;   % Robot 4
    11.5, 5.0,  pi;     % Robot 5
    11.0, 9.5, -pi/2;   % Robot 6
];

waypoints = cell(num_robots, 1);
waypoints{1} = [3, 8, 7, 4, 2, 1, 6, 5,  1, 2, 5, 7, 8, 3, 4, 6,  5, 6, 1, 3, 4, 7, 8, 2,  1];
waypoints{2} = [1, 2, 5, 7, 8, 3, 4, 6,  7, 4, 3, 8, 6, 5, 2, 1,  6, 7, 2, 5, 3, 8, 1, 4,  1];
waypoints{3} = [2, 3, 6, 1, 7, 4, 5, 8,  6, 7, 2, 5, 3, 8, 1, 4,  7, 4, 3, 8, 6, 5, 2, 1,  1];
waypoints{4} = [7, 4, 3, 8, 6, 5, 2, 1,  5, 6, 1, 3, 4, 7, 8, 2,  1, 2, 5, 7, 8, 3, 4, 6,  1];
waypoints{5} = [5, 6, 1, 3, 4, 7, 8, 2,  3, 8, 7, 4, 2, 1, 6, 5,  2, 3, 6, 1, 7, 4, 5, 8,  1];
waypoints{6} = [6, 7, 2, 5, 3, 8, 1, 4,  2, 3, 6, 1, 7, 4, 5, 8,  3, 8, 7, 4, 2, 1, 6, 5,  1];

for i = 1:num_robots
    robots(i).id                 = i;
    robots(i).length              = robot_params.length;
    robots(i).width               = robot_params.width;
    robots(i).max_v               = robot_params.max_v;
    robots(i).max_w               = robot_params.max_w;
    robots(i).v                   = 0;
    robots(i).w                   = 0;
    robots(i).detected_obstacles  = [];
    robots(i).task_done           = 0;
    robots(i).pose                = initial_poses(i,:);
    robots(i).target_poses        = tasks(waypoints{i}, :);
    robots(i).current_target_idx  = 1;
    robots(i).hold_circ           = false;
    robots(i).d0_g                = inf;
end

%% 10. Simulation Parameters & Data Storage

dt         = 0.01;              % simulation time step, s
t_max      = 200.0;             % maximum simulation time, s
time_steps = 0:dt:t_max;
n_steps    = length(time_steps);
pos_tol    = 0.10;              % position arrival tolerance tol_p, m
ang_tol    = pi/180;            % heading arrival tolerance tol_phi, rad (1 deg)

trajectory_history       = cell(num_robots, 1);   % pose history [x, y, phi] per step
sensor_history            = cell(num_robots, 1);   % LiDAR hit points per step
task_history              = cell(num_robots, 1);   % waypoint target [x, y] per step
subtarget_history         = cell(num_robots, 1);   % path prediction sub-target [x, y] per step
velocity_history          = cell(num_robots, 1);   % [v, w] per step
human_trajectory_history  = cell(num_humans, 1);   % human position [x, y] per step

for i = 1:num_robots
    trajectory_history{i}      = zeros(n_steps, 3);
    trajectory_history{i}(1,:) = robots(i).pose;
    sensor_history{i}          = cell(n_steps, 1);
    task_history{i}            = zeros(n_steps, 2);
    subtarget_history{i}       = zeros(n_steps, 2);
    velocity_history{i}        = zeros(n_steps, 2);
end

for h = 1:num_humans
    human_trajectory_history{h}      = zeros(n_steps, 2);
    human_trajectory_history{h}(1,:) = humans(h).pose;
end

%% 11. Initial LiDAR Scan

roa = buildRobotObsArr(robots, num_robots);
hoa = buildHumanObsArr(humans, num_humans);

for i = 1:num_robots
    det = simulateLidar(robots(i), lidar_wall_segs, lidar_circle_obs, ...
                        roa, hoa, i, lidar_params, robot_params);
    robots(i).detected_obstacles = det;
    sensor_history{i}{1}         = det;
end

%% 12. Main Simulation Loop

fprintf('\n=== A_Data_Sim_Multi_Robot ===\n');
fprintf('    Robots : %d  |  Humans : %d  |  dt : %.3f s  |  t_max : %.1f s\n', ...
    num_robots, num_humans, dt, t_max);
fprintf('    mu_obstacle=%.3f   mu_robot=%.3f   lambda=%.1f\n\n', ...
    ctrl_params.mu_obstacle, ctrl_params.mu_robot, ctrl_params.lambda);

for step = 2:n_steps
    t = time_steps(step);

    % -- Early exit when all robots reach their goals --
    if all([robots.task_done] == 1)
        n_steps = step - 1;
        fprintf('  All robots done at t = %.2f s\n', t);
        break;
    end

    % -- Update human positions (ping-pong between target1 and target2) --
    for h = 1:num_humans
        cur_pos = humans(h).pose;
        if humans(h).direction == 1
            tgt_pos = humans(h).target1;
        else
            tgt_pos = humans(h).target2;
        end

        dxh = tgt_pos(1) - cur_pos(1);
        dyh = tgt_pos(2) - cur_pos(2);
        dh  = norm([dxh, dyh]);

        if dh < 0.1
            humans(h).direction = 3 - humans(h).direction;   % toggle 1 <-> 2
        else
            step_dir       = [dxh, dyh] / dh;
            move_dist      = min(humans(h).speed * dt, dh);
            humans(h).pose = cur_pos + step_dir * move_dist;
        end

        human_trajectory_history{h}(step,:) = humans(h).pose;
    end

    % -- Build robot and human obstacle arrays for this step --
    robots_obs_arr = buildRobotObsArr(robots, num_robots);
    human_obs_arr  = buildHumanObsArr(humans, num_humans);

    for i = 1:num_robots

        % -- LiDAR scan at 10 Hz --
        lidar_step_interval = round(1 / (lidar_params.frequency * dt));
        if mod(step - 1, lidar_step_interval) == 0
            det = simulateLidar(robots(i), lidar_wall_segs, lidar_circle_obs, ...
                                robots_obs_arr, human_obs_arr, i, lidar_params, robot_params);
            robots(i).detected_obstacles = det;
            sensor_history{i}{step}      = det;
        else
            robots(i).detected_obstacles = sensor_history{i}{step-1};
            sensor_history{i}{step}      = sensor_history{i}{step-1};
        end

        % -- Current target pose (next waypoint, or parking pose if done) --
        tgt_idx = robots(i).current_target_idx;
        if tgt_idx < size(robots(i).target_poses, 1)
            tpose = robots(i).target_poses(tgt_idx, :);
        else
            tpose = initial_poses(i, :);
        end
        task_history{i}(step,:) = tpose(1:2);

        % -- Arrival check --
        pos_err = norm(tpose(1:2) - robots(i).pose(1:2));
        ang_err = abs(angdiff(tpose(3), robots(i).pose(3)));
        if pos_err < pos_tol && ang_err < ang_tol
            if tgt_idx < size(robots(i).target_poses, 1)
                robots(i).current_target_idx = tgt_idx + 1;
                fprintf('  R%d reached waypoint %d at t = %.2f s\n', i, tgt_idx, t);
            else
                robots(i).v = 0;
                robots(i).w = 0;
                trajectory_history{i}(step,:)  = robots(i).pose;
                subtarget_history{i}(step,:)   = tpose(1:2);
                if ~robots(i).task_done
                    robots(i).task_done = 1;
                    fprintf('  R%d parked at t = %.2f s\n', i, t);
                end
                continue;
            end
        end

        % -- Path Prediction --
        if pos_err > path_pred.dist_min
            % -- Forward-simulate robot to predict future position --
            robots_pred = robots(i);
            for pred_time = 1:path_pred.n_steps
                LgV  = computeDerivCLFPosition(robots_pred, tpose);
                LgBL_tilde = computeDerivCBFLidar(robots_pred, ctrl_params, robot_params);
                if cbf_cr_enabled
                    LgBj_tilde = computeDerivCBFRobot(robots_pred, i, robots_obs_arr, ctrl_params, robot_params);
                else
                    LgBj_tilde = [0, 0];
                end
                LgB_tilde  = LgBL_tilde + LgBj_tilde;
                LgW    = computeDerivCLBF(ctrl_params, LgV, LgB_tilde);
                u_clbf = computeCLBFController(ctrl_params, LgW);
                robots_pred.pose(1) = robots_pred.pose(1) + u_clbf(1) * path_pred.t_sample;
                robots_pred.pose(2) = robots_pred.pose(2) + u_clbf(2) * path_pred.t_sample;
            end
            tpose(1:2) = robots_pred.pose(1:2);   % shift CLF target to predicted position
        end

        % -- Log path-prediction sub-target --
        subtarget_history{i}(step,:) = tpose(1:2);

        % -- Compute CLBF derivatives and control input --
        LgV  = computeDerivCLFPosition(robots(i), tpose);
        LgBL_tilde = computeDerivCBFLidar(robots(i), ctrl_params, robot_params);
        if cbf_cr_enabled
            LgBj_tilde = computeDerivCBFRobot(robots(i), i, robots_obs_arr, ctrl_params, robot_params);
        else
            LgBj_tilde = [0, 0];
        end
        LgB_tilde  = LgBL_tilde + LgBj_tilde;
        LgW = computeDerivCLBF(ctrl_params, LgV, LgB_tilde);
        u_p = computeCLBFController(ctrl_params, LgW);

        % -- Heading controller --
        u_phi = computeHeadingController(robots(i), tpose, ctrl_params);

        % -- Compute linear and angular velocities --
        v = computeLinearVelocity(robots(i),  robot_params, u_p, pos_tol, pos_err);
        w = computeAngularVelocity(robots(i), robot_params, u_p, u_phi, ...
                                   pos_tol, ang_tol, pos_err, ang_err);

        % -- Integrate robot pose one step forward --
        robots(i).v = v;
        robots(i).w = w;
        robots(i)   = updateRobotPose(robots(i), dt);

        % -- Log trajectory and velocity --
        trajectory_history{i}(step,:) = robots(i).pose;
        velocity_history{i}(step,:)   = [v, w];

    end  % robot loop

    if mod(step, 100) == 0
        fprintf('  %.1f %%   t = %.1f s\n', step/n_steps*100, t);
    end

end  % simulation loop
fprintf('\nSimulation complete.  Steps recorded : %d   (%.2f s)\n', ...
    n_steps, time_steps(n_steps));

%% 13. Save .mat

saved_sensor_history = sensor_history{saved_sensor_robot};

save('sim_results.mat', ...
    'trajectory_history', 'saved_sensor_history', 'saved_sensor_robot', ...
    'task_history', 'subtarget_history', 'velocity_history', 'human_trajectory_history', ...
    'n_steps', 'time_steps', 'dt', 't_max',                              ...
    'warehouse', 'robot_params', 'ctrl_params', 'lidar_params', 'path_pred', ...
    'tasks', 'docks', 'zones', 'lidar_wall_segs', 'lidar_circle_obs',    ...
    'initial_poses', 'waypoints', 'num_robots', 'num_tasks', 'num_humans', ...
    'robots', 'humans', 'cbf_cr_enabled');

fprintf('\nSaved : sim_results.mat (sensor history : Robot %d only)\n', saved_sensor_robot);
fprintf('Load in A_Plot_Main_Multi_Robot.m to animate / record video.\n');

%% 14. Trajectory Plot

fig1 = figure('Name', 'Multi-Robot Warehouse - Trajectory', ...
    'Position', [1600, 100, 1200, 900], 'Color', 'w');
hold on; grid off; axis equal;
set(gca, 'FontSize', 16, 'LineWidth', 1.0);
xlabel('$x$ Position (m)', 'FontSize', 24, 'Interpreter', 'latex');
ylabel('$y$ Position (m)', 'FontSize', 24, 'Interpreter', 'latex');
title('Multi-Robot Warehouse Test Trajectories', 'FontSize', 24, ...
    'FontWeight', 'bold', 'FontName', 'Times New Roman');

% -- Warehouse boundary --
rectangle('Position', [0, 0, warehouse.x_size, warehouse.y_size], ...
          'EdgeColor', 'k', 'LineWidth', 2);

% -- Docks --
for k = 1:size(docks, 1)
    dx_k = docks(k,1) - docks(k,3)/2;
    dy_k = docks(k,2) - docks(k,4)/2;
    rectangle('Position', [dx_k, dy_k, docks(k,3), docks(k,4)], ...
              'FaceColor', [0.8, 0.8, 0.8], 'EdgeColor', [0.3, 0.3, 0.3], 'LineWidth', 1.5);
    text(docks(k,1), docks(k,2), sprintf('D_%d', k), ...
         'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'FontSize', 12);
end

% -- Task locations --
for k = 1:num_tasks
    plot(tasks(k,1), tasks(k,2), 'p', 'MarkerSize', 12, ...
         'MarkerFaceColor', [0.3, 0.3, 0.3], 'MarkerEdgeColor', 'k', 'LineWidth', 1.5);
end

% -- Static circle obstacles --
theta_circ = linspace(0, 2*pi, 64);
for k = 1:size(lidar_circle_obs, 1)
    xc = lidar_circle_obs(k,1) + lidar_circle_obs(k,3) * cos(theta_circ);
    yc = lidar_circle_obs(k,2) + lidar_circle_obs(k,3) * sin(theta_circ);
    fill(xc, yc, [0.3, 0.3, 0.3], 'EdgeColor', 'k', 'LineWidth', 1.5);
end

% -- Human paths and start positions --
human_color = [1, 0.5, 0];
for h = 1:num_humans
    plot(human_trajectory_history{h}(1:n_steps,1), human_trajectory_history{h}(1:n_steps,2), ...
         '--', 'Color', human_color, 'LineWidth', 2);
    xc = humans(h).start_pos(1) + humans(h).radius * cos(theta_circ);
    yc = humans(h).start_pos(2) + humans(h).radius * sin(theta_circ);
    fill(xc, yc, human_color, 'EdgeColor', 'k', 'LineWidth', 1.5);
end

% -- Robot trajectories and parking spots --
traj_colors = lines(num_robots);
for i = 1:num_robots
    plot(trajectory_history{i}(1:n_steps,1), trajectory_history{i}(1:n_steps,2), ...
         'Color', traj_colors(i,:), 'LineWidth', 2, 'DisplayName', sprintf('Robot %d', i));
    plot(initial_poses(i,1), initial_poses(i,2), 'o', 'MarkerSize', 10, ...
         'MarkerFaceColor', traj_colors(i,:), 'MarkerEdgeColor', 'k', 'LineWidth', 2);
end

xlim([-0.5, warehouse.x_size+0.5]);
ylim([-0.5, warehouse.y_size+0.5]);
hold off;

%% LOCAL FUNCTIONS

% Compute CLF position gradient: LgV = [x - x_goal, y - y_goal]
% Eq. (8) & (9)
function LgV = computeDerivCLFPosition(robot, target_pose)
    LgV = [robot.pose(1) - target_pose(1), ...
           robot.pose(2) - target_pose(2)];
end

% Compute CBF gradient for LiDAR-detected obstacles
% Eq. (20)
function LgBL_tilde = computeDerivCBFLidar(robot, ctrl_params, robot_params)
    x  = robot.pose(1);  y = robot.pose(2);
    l  = robot_params.length;
    mu = ctrl_params.mu_obstacle;
    r  = l/2;

    LgBL_tilde = [0, 0];
    det  = robot.detected_obstacles;
    if ~isempty(det)
        dl = (r + mu)^2 - (r)^2;
        for k = 1:size(det, 1)
            dkx  = x - det(k,1);
            dky  = y - det(k,2);
            Bl   = -(dkx^2 + dky^2) + (r)^2;
            LgBL = [-2*dkx, -2*dky];
            rho  = cosineBlend(Bl, dl);
            LgBL_tilde = LgBL_tilde + rho * LgBL;
        end
    end
end

% Compute CBF gradient for robot-robot conflict resolution (priority-based)
% Eq. (21)
function LgBj_tilde = computeDerivCBFRobot(robot, robot_idx, robots_obs_arr, ctrl_params, robot_params)
    % Lower index = higher priority.
    % robot_idx > j -> this robot yields -> use mu_robot (larger margin)
    % robot_idx < j -> higher priority   -> use mu_obstacle (standard margin)
    x        = robot.pose(1);  y = robot.pose(2);
    l        = robot_params.length;
    mu_obs   = ctrl_params.mu_obstacle;
    mu_robot = ctrl_params.mu_robot;
    r        = l/2;

    LgBj_tilde = [0, 0];
    for j = 1:size(robots_obs_arr, 1)
        if j == robot_idx, continue; end
        djx = x - robots_obs_arr(j,1);
        djy = y - robots_obs_arr(j,2);
        rc  = r + robots_obs_arr(j,4);   % combined robot radius

        Br = -(djx^2 + djy^2) + rc^2;
        % Eq. (17)
        if robot_idx > j
            mu = mu_robot;   % lower priority -> larger safety margin
        else
            mu = mu_obs;     % higher priority -> standard margin
        end
        dr   = (rc + mu)^2 - rc^2;
        LgBj = [-2*djx, -2*djy];
        rho  = cosineBlend(Br, dr);
        LgBj_tilde = LgBj_tilde + rho * LgBj;
    end
end

% Compute CLBF gradient: LgW = LgV + lambda * LgB
% Eq. (22)
function LgW = computeDerivCLBF(ctrl_params, LgV, LgB_tilde)
    LgW = LgV + ctrl_params.lambda * LgB_tilde;
end

% Compute CLBF control input: u_p = -sqrt(gamma) * LgW
% Eq. (23)
function u_clbf = computeCLBFController(ctrl_params, LgW)
    u_clbf = -sqrt(ctrl_params.gamma) * LgW;
end

% Compute heading CLF control input: u_phi = -sqrt(gamma_phi) * (phi - phi_goal)
% Eq. (25)
function u_phi = computeHeadingController(robot, target_pose, ctrl_params)
    e_phi = angdiff(robot.pose(3), target_pose(3));
    u_phi = -sqrt(ctrl_params.gamma_phi) * e_phi;
end

% Compute linear velocity via NID transformation, clamped to max_v
% Eq. (24)
function v = computeLinearVelocity(robot, robot_params, u_p, pos_tol, pos_err)
    phi = robot.pose(3);
    if pos_err >= pos_tol
        v = u_p(1)*cos(phi) + u_p(2)*sin(phi);
    else
        v = 0;
    end
    v = max(-robot_params.max_v, min(robot_params.max_v, v));
end

% Compute angular velocity via NID transformation, clamped to max_w
% Eq. (26)
function w = computeAngularVelocity(robot, robot_params, u_p, u_phi, ...
        pos_tol, ang_tol, pos_err, ang_err)
    phi   = robot.pose(3);
    width = robot_params.width;
    if pos_err >= pos_tol
        w = (2/width) * (-u_p(1)*sin(phi) + u_p(2)*cos(phi));
    else
        if ang_err >= ang_tol
            w = u_phi;
        else
            w = 0;
        end
    end
    w = max(-robot_params.max_w, min(robot_params.max_w, w));
end

% Simulate LiDAR ray-cast and return all hit coordinates [N x 2]
function det = simulateLidar(robot, wall_segs, circle_obs, robots_obs, human_obs, ...
        robot_idx, lidar_params, robot_params)
    % Detects in priority order: walls -> circles -> other robots (ellipse) -> humans (circle)
    x   = robot.pose(1);
    y   = robot.pose(2);
    phi = robot.pose(3);
    det = [];
    as  = 2*pi / lidar_params.num_samples;

    for ang = 0 : as : (2*pi - as)
        ra  = phi + ang;
        rdx = cos(ra);
        rdy = sin(ra);
        md  = lidar_params.range;
        hp  = [];

        % -- Wall segments --
        for s = 1:size(wall_segs, 1)
            ws = wall_segs(s,:);
            [hit, d, hx, hy] = checkSeg(x, y, rdx, rdy, ws(1), ws(2), ws(3), ws(4));
            if hit && d < md && d > lidar_params.min_detect_dist
                md = d; hp = [hx, hy];
            end
        end

        % -- Circular obstacles --
        for k = 1:size(circle_obs, 1)
            [hit, d, hx, hy] = checkCircle(x, y, rdx, rdy, ...
                circle_obs(k,1), circle_obs(k,2), circle_obs(k,3));
            if hit && d < md && d > lidar_params.min_detect_dist
                md = d; hp = [hx, hy];
            end
        end

        % -- Other robots (ellipse approximation) --
        for j = 1:size(robots_obs, 1)
            if j == robot_idx, continue; end
            [hit, d, hx, hy] = checkEllipse(x, y, rdx, rdy, ...
                robots_obs(j,1), robots_obs(j,2), robots_obs(j,3), ...
                robot_params.length, robot_params.width);
            if hit && d < md && d > lidar_params.min_detect_dist
                md = d; hp = [hx, hy];
            end
        end

        % -- Human obstacles (circle approximation) --
        for k = 1:size(human_obs, 1)
            [hit, d, hx, hy] = checkCircle(x, y, rdx, rdy, ...
                human_obs(k,1), human_obs(k,2), human_obs(k,3));
            if hit && d < md && d > lidar_params.min_detect_dist
                md = d; hp = [hx, hy];
            end
        end

        if ~isempty(hp)
            det = [det; hp]; %#ok<AGROW>
        end
    end

    if ~isempty(det)
        det = removeDuplicatePts(det, 0.05);
    end
end

% Ray-segment intersection check; includes endpoint cap detection
function [hit, td, ix, iy] = checkSeg(rx, ry, rdx, rdy, x1, y1, x2, y2)
    hit = false;  td = inf;  ix = 0;  iy = 0;
    sx  = x2 - x1;
    sy  = y2 - y1;
    den = rdx*sy - rdy*sx;

    if abs(den) >= 1e-10
        t_hit = ((x1-rx)*sy - (y1-ry)*sx) / den;
        u_hit = ((x1-rx)*rdy - (y1-ry)*rdx) / den;

        if t_hit > 1e-9 && u_hit >= 0 && u_hit <= 1
            ix  = x1 + u_hit * sx;
            iy  = y1 + u_hit * sy;
            td  = sqrt((ix-rx)^2 + (iy-ry)^2);
            hit = true;
            return;
        end
    end

    ep_radius = 0.05;
    endpoints = [x1, y1; x2, y2];

    for k = 1:2
        epx  = endpoints(k,1);
        epy  = endpoints(k,2);
        t_ep = (epx - rx)*rdx + (epy - ry)*rdy;
        if t_ep <= 1e-9, continue; end
        foot_x = rx + t_ep*rdx;
        foot_y = ry + t_ep*rdy;
        perp_d = sqrt((foot_x - epx)^2 + (foot_y - epy)^2);
        if perp_d < ep_radius
            d_ep = sqrt((epx - rx)^2 + (epy - ry)^2);
            if d_ep < td
                td  = d_ep;
                ix  = epx;
                iy  = epy;
                hit = true;
            end
        end
    end
end

% Ray-circle intersection check
function [hit, dist, hx, hy] = checkCircle(rx, ry, rdx, rdy, cx, cy, r)
    hit = false; dist = inf; hx = 0; hy = 0;
    fx = rx-cx; fy = ry-cy;
    a  = rdx^2 + rdy^2;
    b  = 2*(fx*rdx + fy*rdy);
    c  = fx^2 + fy^2 - r^2;
    disc = b^2 - 4*a*c;
    if disc < 0, return; end
    sq = sqrt(disc);
    t  = min((-b-sq)/(2*a), (-b+sq)/(2*a));
    if t > 0
        hit = true; dist = t; hx = rx+t*rdx; hy = ry+t*rdy;
    end
end

% Ray-ellipse intersection check (robot body approximation)
function [hit, dist, hx, hy] = checkEllipse(rx, ry, rdx, rdy, cx, cy, cth, elen, ewid)
    hit = false; dist = inf; hx = 0; hy = 0;
    a = elen/2; b = ewid/2;
    rxl = rx-cx; ryl = ry-cy;
    ct = cos(-cth); st = sin(-cth);
    rr = ct*rxl - st*ryl;  rs = st*rxl + ct*ryl;
    dr = ct*rdx - st*rdy;  ds = st*rdx + ct*rdy;
    A  = (dr/a)^2 + (ds/b)^2;
    B2 = 2*((rr*dr)/a^2 + (rs*ds)/b^2);
    C  = (rr/a)^2 + (rs/b)^2 - 1;
    disc = B2^2 - 4*A*C;
    if disc < 0, return; end
    sq = sqrt(disc);
    t1 = (-B2-sq)/(2*A);  t2 = (-B2+sq)/(2*A);
    if     t1 > 0 && t2 > 0, t = min(t1,t2);
    elseif t1 > 0,            t = t1;
    elseif t2 > 0,            t = t2;
    else,                     return;
    end
    hit = true; dist = t; hx = rx+t*rdx; hy = ry+t*rdy;
end

% Remove duplicate or near-duplicate LiDAR hit points within threshold distance
function pts = removeDuplicatePts(pts, thresh)
    if isempty(pts), return; end
    keep = pts(1,:);
    for i = 2:size(pts,1)
        if all(vecnorm(keep - pts(i,:), 2, 2) >= thresh)
            keep = [keep; pts(i,:)]; %#ok<AGROW>
        end
    end
    pts = keep;
end

% Build N x 4 obstacle matrix [x, y, phi, half_length] from robot struct array
function arr = buildRobotObsArr(robots, n)
    arr = zeros(n, 4);
    for i = 1:n
        arr(i,:) = [robots(i).pose(1:2), robots(i).pose(3), robots(i).length/2];
    end
end

% Build N x 3 obstacle matrix [x, y, radius] from human struct array
function arr = buildHumanObsArr(humans, n)
    arr = zeros(n, 3);
    for h = 1:n
        arr(h,:) = [humans(h).pose, humans(h).radius];
    end
end

% Integrate robot unicycle pose one time step forward
function robot = updateRobotPose(robot, dt)
    phi = robot.pose(3);
    robot.pose(1) = robot.pose(1) + robot.v * cos(phi) * dt;
    robot.pose(2) = robot.pose(2) + robot.v * sin(phi) * dt;
    robot.pose(3) = atan2(sin(robot.pose(3) + robot.w*dt), ...
                          cos(robot.pose(3) + robot.w*dt));
end

% Compute signed angle difference phi1 - phi2, wrapped to [-pi, pi]
function delta = angdiff(phi1, phi2)
    delta = atan2(sin(phi1-phi2), cos(phi1-phi2));
end

% Cosine blend function: smooth transition between 0 and 1 over interval [-eta, 0]
function alpha = cosineBlend(sigma, eta)
    if     sigma >= 0,      alpha = 1;
    elseif sigma <= -eta,   alpha = 0;
    else,  alpha = 0.5 * (cos((sigma / eta) * pi) + 1);
    end
end
