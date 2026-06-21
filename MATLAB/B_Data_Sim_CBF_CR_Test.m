%% B_Data_Sim_CBF_CR_Test.m
% Simulation : 2-Robot CBF-Based Conflict Resolution (CBF-CR) Test
% Output     : CBF_CR_Test.mat
% Run this script first before B_Plot_CBF_CR_Test.m

clear; clc;

%% 0. User Configuration

% -- CBF-CR Toggle --
% true  -> mu_robot > mu_obstacle (priority-based safety margin active)
% false -> use LiDAR-CBF only (robots detected via LiDAR, no robot CBF term)
cbf_cr_enabled = false;

% -- Robot Start & Goal Poses  [x, y, phi] --
r1_start = [0.0, 0.0, 0.0];    % Robot 1 start pose
r1_goal  = [6.0, 0.0, 0.0];    % Robot 1 goal pose
r2_start = [6.0, 0.0,  pi];    % Robot 2 start pose
r2_goal  = [0.0, 0.0,  pi];    % Robot 2 goal pose

% -- Axis Limits --
% axis_manual = [-2.0, 10.0, -5.0, 9.0];   % [xmin xmax ymin ymax]

%% 1. Robot Physical Parameters

robot_params.length = 0.60;    % ellipse major axis l (front-to-back), m
robot_params.width  = 0.40;    % ellipse minor axis w (side-to-side), m
robot_params.max_v  = 1.6;     % maximum linear speed v_max, m/s
robot_params.max_w  = pi;      % maximum angular speed w_max, rad/s

%% 2. Controller Parameters

ctrl_params.gamma     = 16.0;    % CLF position gain gamma
ctrl_params.gamma_phi =  9.0;    % CLF heading gain gamma_phi
ctrl_params.lambda    =  5.0;    % CBF weight lambda
ctrl_params.mu_obstacle =  0.15;    %  safety margin mu_o - static obstacles & LiDAR hits, m
ctrl_params.mu_robot    =  0.35;    % safety margin mu_r (mu_r >= 2*mu_o) - robot-robot, m

%% 3. LiDAR Parameters

lidar_params.enabled         = true;
lidar_params.range           = 12.0;  % detection range, m
lidar_params.num_samples     = 800;   % total ray samples per scan (360 deg coverage)
lidar_params.min_detect_dist = 0.1;   % minimum valid hit distance, m
lidar_params.frequency       = 10;    % scan frequency, hz

%% 4. Path Prediction Parameters

path_pred.n_steps  = 5;      % forward-simulation horizon n_steps, steps
path_pred.t_sample = 0.05;   % integration step per prediction step t_sample, s
path_pred.dist_min = 1.0;    % activate only when distance to goal e_p > dist_min eps_p, m

%% 5. Scenario Setup

num_robots       = 2;
lidar_wall_segs  = [];     % no wall obstacles in this scenario
lidar_circle_obs = [];     % no circle obstacles in this scenario

%% 6. Robot Initialisation

r_starts = {r1_start, r2_start};
r_goals  = {r1_goal,  r2_goal};

for i = 1:num_robots
    robots(i).id                 = i;
    robots(i).length             = robot_params.length;
    robots(i).width              = robot_params.width;
    robots(i).max_v              = robot_params.max_v;
    robots(i).max_w              = robot_params.max_w;
    robots(i).v                  = 0;
    robots(i).w                  = 0;
    robots(i).detected_obstacles = [];
    robots(i).task_done          = 0;
    robots(i).pose               = r_starts{i};
    robots(i).target_poses       = r_goals{i};   % [x, y, phi]
    robots(i).hold_circ          = false;
    robots(i).d0_g               = inf;
end

initial_poses = zeros(num_robots, 3);
for i = 1:num_robots
    initial_poses(i,:) = r_starts{i};
end

%% 7. Simulation Parameters & Data Storage

dt         = 0.01;              % simulation time step, s
t_max      = 10.0;              % maximum simulation time, s
time_steps = 0:dt:t_max;
n_steps    = length(time_steps);
pos_tol    = 0.10;              % position arrival tolerance tol_p, m
ang_tol    = pi/180;            % heading arrival tolerance tol_phi, rad (1 deg)

trajectory_history = cell(num_robots, 1);   % pose history [x, y, phi] per step
sensor_history     = cell(num_robots, 1);   % LiDAR hit points per step
velocity_history   = cell(num_robots, 1);   % [v, w] per step
a_history          = zeros(n_steps, 4);

for i = 1:num_robots
    trajectory_history{i}      = zeros(n_steps, 3);
    trajectory_history{i}(1,:) = robots(i).pose;
    sensor_history{i}          = cell(n_steps, 1);
    velocity_history{i}        = zeros(n_steps, 2);
end

%% 8. Initial LiDAR Scan

roa = buildRobotObsArr(robots, num_robots);

for i = 1:num_robots
    det = simulateLidar(robots(i), lidar_wall_segs, lidar_circle_obs, ...
                        roa, i, lidar_params, robot_params);
    robots(i).detected_obstacles = det;
    sensor_history{i}{1}         = det;
end

%% 9. Main Simulation Loop

fprintf('\n=== B_Data_Sim_CBF_CR_Test ===\n');
fprintf('    Robots : %d  |  dt : %.3f s  |  t_max : %.1f s\n', num_robots, dt, t_max);
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

    % -- Build robot obstacle array for CBF-CR --
    robots_obs_arr = buildRobotObsArr(robots, num_robots);

    for i = 1:num_robots

        % -- Hold pose if this robot has finished --
        if robots(i).task_done
            trajectory_history{i}(step,:) = robots(i).pose;
            continue;
        end

        tpose = robots(i).target_poses;   % [x, y, phi] goal

        % -- LiDAR scan at 10 Hz --
        lidar_step_interval = round(1 / (lidar_params.frequency * dt));
        if mod(step - 1, lidar_step_interval) == 0
            det = simulateLidar(robots(i), lidar_wall_segs, lidar_circle_obs, ...
                                robots_obs_arr, i, lidar_params, robot_params);
            robots(i).detected_obstacles = det;
            sensor_history{i}{step}      = det;
        else
            robots(i).detected_obstacles = sensor_history{i}{step-1};
            sensor_history{i}{step}      = sensor_history{i}{step-1};
        end

        % -- Arrival check --
        pos_err = norm(tpose(1:2) - robots(i).pose(1:2));
        ang_err = abs(angdiff(tpose(3), robots(i).pose(3)));
        if pos_err < pos_tol && ang_err < ang_tol
            robots(i).task_done = 1;
            fprintf('  R%d reached goal at t = %.2f s\n', i, t);
        end

        % -- Path Prediction --
        % Algorithm 1 & Fig. 3
        if pos_err > path_pred.dist_min
            % -- Forward-simulate robot pose to predict future position --
            robots_pred = robots(i);
            for pred_time = 1:path_pred.n_steps
                LgV = computeDerivCLFPosition(robots_pred, tpose);
                LgBL_tilde = computeDerivCBFLidar(robots_pred, ctrl_params, robot_params);
                if cbf_cr_enabled
                    LgBj_tilde = computeDerivCBFRobot(robots_pred, i, robots_obs_arr, ctrl_params, robot_params);
                else
                    LgBj_tilde = [0, 0];
                end
                LgB_tilde  = LgBL_tilde + LgBj_tilde;
                LgW = computeDerivCLBF(ctrl_params, LgV, LgB_tilde);
                u_clbf = computeCLBFController(ctrl_params, LgW);
                robots_pred.pose(1) = robots_pred.pose(1) + u_clbf(1) * path_pred.t_sample;
                robots_pred.pose(2) = robots_pred.pose(2) + u_clbf(2) * path_pred.t_sample;
            end
            tpose(1:2) = robots_pred.pose(1:2);   % shift CLF target to predicted pose
        end

        % -- Compute CLBF derivatives and control input --
        LgV = computeDerivCLFPosition(robots(i), tpose);
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

%% 10. Distance Travelled per Robot

fprintf('\n--- Distance Travelled per Robot ---\n');
for i = 1:num_robots
    traj  = trajectory_history{i}(1:n_steps, 1:2);
    diffs = diff(traj, 1, 1);
    dist  = sum(sqrt(sum(diffs.^2, 2)));
    fprintf('  R%d : %.4f m\n', i, dist);
end

%% 11. Trajectory Plot

% -- Compute auto axis limits from trajectory extents --
all_x = []; all_y = [];
for i = 1:num_robots
    traj  = trajectory_history{i}(1:n_steps, :);
    all_x = [all_x; traj(:,1)]; %#ok<AGROW>
    all_y = [all_y; traj(:,2)]; %#ok<AGROW>
end
pad         = 1.5;
axis_limits = [min(all_x)-pad, max(all_x)+pad, min(all_y)-pad, max(all_y)+pad];
% axis_limits = axis_manual;   % uncomment for manual axis (define in Section 0)

% -- Figure setup --
fig1 = figure('Name', 'CLBF Test - Trajectory', ...
    'Position', [50, 100, 960, 700], 'Color', 'w');
ax1 = axes('Parent', fig1);
hold(ax1, 'on'); grid(ax1, 'on'); axis(ax1, 'equal');
set(ax1, 'FontSize', 16, 'FontName', 'Times New Roman', 'LineWidth', 1.0);
xlabel(ax1, '$x$ Position (m)', 'FontSize', 22, 'Interpreter', 'latex');
ylabel(ax1, '$y$ Position (m)', 'FontSize', 22, 'Interpreter', 'latex');
title(ax1, sprintf('2-Robot CBF-CR Test'), ...
    'FontSize', 18, 'FontWeight', 'bold', 'FontName', 'Times New Roman');
xlim(ax1, axis_limits(1:2));
ylim(ax1, axis_limits(3:4));

% -- Trajectories, heading arrows, start & goal markers --
traj_colors = lines(max(num_robots, 4));
for i = 1:num_robots
    traj = trajectory_history{i}(1:n_steps, :);
    cl   = traj_colors(i, :);

    % Trajectory line
    plot(ax1, traj(:,1), traj(:,2), '-', 'Color', cl, 'LineWidth', 2.2, ...
        'DisplayName', sprintf('$R_{%d}$ trajectory', i));

    % Start marker
    plot(ax1, traj(1,1), traj(1,2), 'o', 'MarkerSize', 11, ...
        'MarkerFaceColor', cl*0.5, 'MarkerEdgeColor', cl, 'LineWidth', 2, ...
        'HandleVisibility', 'off');
    text(ax1, traj(1,1)-0.35, traj(1,2)+0.35, sprintf('$R_{%d}$ Start', i), ...
        'FontSize', 11, 'Color', cl, 'Interpreter', 'latex', ...
        'FontName', 'Times New Roman', 'FontWeight', 'bold');

    % Goal marker
    gp = robots(i).target_poses;
    plot(ax1, gp(1), gp(2), 'p', 'MarkerSize', 18, ...
        'MarkerFaceColor', cl, 'MarkerEdgeColor', 'k', 'LineWidth', 1.5, ...
        'DisplayName', sprintf('$R_{%d}$ Goal', i));
    text(ax1, gp(1)+0.25, gp(2)+0.35, sprintf('$R_{%d}$ Goal', i), ...
        'FontSize', 11, 'Color', cl, 'Interpreter', 'latex', ...
        'FontName', 'Times New Roman', 'FontWeight', 'bold');
end

legend(ax1, 'show', 'Location', 'northeast', 'FontSize', 13, ...
    'FontName', 'Times New Roman', 'Interpreter', 'latex', ...
    'Orientation', 'Horizontal');
hold off;

%% 12. Save .mat

mat_filename = 'CBF_CR_Test.mat';

save(mat_filename, ...
    'trajectory_history', 'sensor_history', 'velocity_history',         ...
    'time_steps', 'n_steps', 'dt', 'num_robots', 'cbf_cr_enabled',      ...
    'robot_params', 'ctrl_params', 'lidar_params', 'path_pred',         ...
    'initial_poses', 'robots');

fprintf('\nSaved : %s\n', mat_filename);
fprintf('Load in B_Plot_CBF_CR_Test.m to animate / record video.\n');

%% LOCAL FUNCTIONS

% Compute CLF position and heading gradient: LgV = [x - x_goal, y - y_goal]
% Eq. (8) & (9)
function LgV = computeDerivCLFPosition(robot, target_pose)
    LgV = [robot.pose(1) - target_pose(1), ...
           robot.pose(2) - target_pose(2)];
end

% Compute LiDAR-CBF gradient
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

% Compute CBF-CR gradient (priority-based)
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
function det = simulateLidar(robot, wall_segs, circle_obs, robots_obs, ...
        robot_idx, lidar_params, robot_params)
    % Detects in priority order: walls -> circles -> other robots (ellipse approx)
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