%% A_Plot_Safety_Distance.m
% Plot : Minimum Robot-to-Obstacle Distance, Multi-Robot Warehouse
% Load : sim_results.mat (run A_Data_Sim_Multi_Robot.m first)
% All display settings are in cfg at the top of this file.

clear; clc;

%% 0. Display Configuration

% -- Mode toggles --
cfg.run_plot = true;     % true = run static figure (system minimum)
cfg.run_anim = false;    % true = run animation (per-robot lines)

% -- Save toggles --
cfg.save_plot     = false;                       % true = export PNG
cfg.save_anim     = false;                       % true = export MP4
cfg.plot_filename = 'Safety_Distance_Plot.png';  % output plot filename
cfg.anim_filename = 'Safety_Distance_Anim.mp4';  % output video filename

% -- Figure size [left, bottom, width, height], px --
cfg.plot_fig_pos = [100, 100, 1200, 675];
cfg.anim_fig_pos = [200, 100, 1200, 675];

% -- Font --
cfg.font_name        = 'Times New Roman';
cfg.font_size_title  = 30;
cfg.font_size_axis   = 24;
cfg.font_size_legend = 12;

% -- Titles (blank by default; set a string to label a specific figure) --
cfg.plot_title = '';
cfg.anim_title = '';

% -- Axis limits --
cfg.y_min_pad = 0.1;    % lower bound padding below the lowest data point, m
cfg.y_max     = 0.4;    % fixed upper bound, m

% -- Animation playback --
cfg.skip_frames = 10;     % render every Nth step (10 -> ~10 fps at dt=0.01)
cfg.anim_speed  = 200.0;  % playback speed multiplier (200 -> 1:1 time ratio at t_max=200s)

%% 1. Load .mat File

mat_filename = 'sim_results.mat';
fprintf('=== A_Plot_Safety_Distance.m ===\n');
if ~exist(mat_filename, 'file')
    error('Required file not found: %s\n  Run A_Data_Sim_Multi_Robot.m first.', mat_filename);
end
d = load(mat_filename);
fprintf('  Loaded : %s\n', mat_filename);

trajectory_history       = d.trajectory_history;
human_trajectory_history = d.human_trajectory_history;
time_steps                = d.time_steps(1:d.n_steps);
n_steps                   = d.n_steps;
num_robots                = d.num_robots;
num_humans                = d.num_humans;
dt                        = d.dt;
warehouse                 = d.warehouse;
lidar_circle_obs          = d.lidar_circle_obs;
robot_params              = d.robot_params;
ctrl_params               = d.ctrl_params;
humans                    = d.humans;

% -- Per-robot colors (lines palette) --
robot_clrs = lines(num_robots);

%% 2. Compute Closest Distances

closest_distances = zeros(num_robots, n_steps);   % [robot, step] -> min distance, m

fprintf('Computing closest distances for all robots...\n');

ea = robot_params.length / 2;   % ellipse semi-major axis
eb = robot_params.width  / 2;   % ellipse semi-minor axis

for step = 1:n_steps

    % -- Current human positions [x, y, radius] --
    human_pos = zeros(num_humans, 3);
    for h = 1:num_humans
        human_pos(h, 1:2) = human_trajectory_history{h}(step, :);
        human_pos(h, 3)   = humans(h).radius;
    end

    for i = 1:num_robots
        pose  = trajectory_history{i}(step, :);
        x     = pose(1);
        y     = pose(2);
        phi   = pose(3);
        min_d = inf;

        % -- Distance to warehouse walls --
        n_samples = 72;
        angles    = linspace(0, 2*pi, n_samples);
        for ang = angles
            lx = ea * cos(ang);
            ly = eb * sin(ang);
            wx = x + lx * cos(phi) - ly * sin(phi);
            wy = y + lx * sin(phi) + ly * cos(phi);
            wall_d = min([wx, warehouse.x_size - wx, wy, warehouse.y_size - wy]);
            min_d  = min(min_d, wall_d);
        end

        % -- Distance to static circle obstacles --
        for k = 1:size(lidar_circle_obs, 1)
            dk    = computeEllipseToCircleDistance(x, y, phi, ea, eb, ...
                lidar_circle_obs(k,1), lidar_circle_obs(k,2), lidar_circle_obs(k,3));
            min_d = min(min_d, dk);
        end

        % -- Distance to other robots (ellipses) --
        for j = 1:num_robots
            if i == j, continue; end
            other = trajectory_history{j}(step, :);
            dj    = computeEllipseToEllipseDistance(x, y, phi, ea, eb, ...
                other(1), other(2), other(3), ea, eb);
            min_d = min(min_d, dj);
        end

        % -- Distance to humans (circles) --
        for h = 1:num_humans
            dh    = computeEllipseToCircleDistance(x, y, phi, ea, eb, ...
                human_pos(h,1), human_pos(h,2), human_pos(h,3));
            min_d = min(min_d, dh);
        end

        closest_distances(i, step) = min_d;
    end

    if mod(step, 500) == 0
        fprintf('  %.1f %%\n', step/n_steps*100);
    end
end

fprintf('Distance computation complete.\n');

system_min = min(closest_distances, [], 1);   % 1 x n_steps, used by static plot

% -- Shared y-axis lower bound --
y_min = max(-0.05, min(system_min) - cfg.y_min_pad);

%% 3. Static Plot (System Minimum)

if cfg.run_plot

    fig1 = figure('Name', 'Safety Distance - System Minimum', ...
        'Position', cfg.plot_fig_pos, 'Color', 'w');
    ax1  = axes('Parent', fig1);
    hold(ax1, 'on'); grid(ax1, 'on');
    set(ax1, 'FontSize', cfg.font_size_axis, 'FontName', cfg.font_name, 'LineWidth', 1.0);
    xlabel(ax1, 'Time (s)', 'FontSize', cfg.font_size_axis, 'Interpreter', 'latex');
    ylabel(ax1, 'Minimum Distance to Obstacle (m)', ...
        'FontSize', cfg.font_size_axis, 'Interpreter', 'latex');
    title(ax1, cfg.plot_title, 'FontSize', cfg.font_size_title, ...
        'FontWeight', 'bold', 'FontName', cfg.font_name);

    % -- System minimum line --
    plot(ax1, time_steps, system_min, 'Color', [0.08, 0.38, 0.65], ...
        'LineWidth', 2.0, 'DisplayName', 'System Minimum');

    % -- CBF active margin reference line --
    plot(ax1, [time_steps(1), time_steps(end)], ...
        [ctrl_params.mu_obstacle, ctrl_params.mu_obstacle], ...
        'r--', 'LineWidth', 2.0, 'DisplayName', 'CBF Active Margin ($\mu$)');

    % -- Zero line --
    plot(ax1, [time_steps(1), time_steps(end)], [0, 0], ...
        'k-', 'LineWidth', 1.0, 'HandleVisibility', 'off');

    xlim(ax1, [0, time_steps(end)]);
    ylim(ax1, [y_min, cfg.y_max]);

    legend(ax1, 'show', 'Location', 'south', 'Orientation', 'horizontal', ...
        'FontSize', cfg.font_size_legend, 'FontName', cfg.font_name, 'Interpreter', 'latex');
    hold off;
    fprintf('Static plot complete.\n');

    if cfg.save_plot
        exportgraphics(fig1, cfg.plot_filename, 'Resolution', 300);
        fprintf('  Plot saved : %s\n', cfg.plot_filename);
    end
end

%% 4. Animation (Per-Robot)

if cfg.run_anim

    % -- Video setup --
    if cfg.save_anim
        vid           = VideoWriter(cfg.anim_filename, 'MPEG-4');
        vid.FrameRate = 10;
        open(vid);
        fprintf('  Recording : %s\n', cfg.anim_filename);
    end

    % -- Figure and axes setup --
    fig2 = figure('Name', 'Safety Distance - Animation', ...
        'Color', 'w', 'Position', cfg.anim_fig_pos);
    ax2  = axes('Parent', fig2);
    hold(ax2, 'on'); grid(ax2, 'off');
    set(ax2, 'FontSize', cfg.font_size_axis, 'FontName', cfg.font_name, 'LineWidth', 1.0);
    xlabel(ax2, 'Time (s)', 'FontSize', cfg.font_size_axis, 'Interpreter', 'latex');
    ylabel(ax2, 'Minimum Distance to Obstacle (m)', ...
        'FontSize', cfg.font_size_axis, 'Interpreter', 'latex');
    title(ax2, cfg.anim_title, 'FontSize', cfg.font_size_title, ...
        'FontWeight', 'bold', 'FontName', cfg.font_name);
    xlim(ax2, [0, time_steps(end)]);
    ylim(ax2, [y_min, cfg.y_max]);

    % -- CBF active margin reference line (static) --
    plot(ax2, [time_steps(1), time_steps(end)], ...
        [ctrl_params.mu_obstacle, ctrl_params.mu_obstacle], ...
        'r--', 'LineWidth', 2.5, 'DisplayName', 'CBF Active Margin ($\mu$)');

    % -- Zero line (static) --
    plot(ax2, [time_steps(1), time_steps(end)], [0, 0], ...
        'k-', 'LineWidth', 1.0, 'HandleVisibility', 'off');

    % -- Pre-allocate per-robot distance lines --
    robot_lines = gobjects(num_robots, 1);
    for i = 1:num_robots
        robot_lines(i) = plot(ax2, NaN, NaN, 'Color', robot_clrs(i,:), ...
            'LineWidth', 1.5, 'DisplayName', sprintf('Robot %d', i));
    end

    legend(ax2, 'show', 'Location', 'south', 'Orientation', 'horizontal', ...
        'FontSize', cfg.font_size_legend, 'FontName', cfg.font_name, 'Interpreter', 'latex');

    h_time = text(ax2, 0.02, 0.97, '', 'Units', 'normalized', ...
        'FontSize', 16, 'FontWeight', 'bold', 'VerticalAlignment', 'top', ...
        'FontName', cfg.font_name, 'BackgroundColor', [1, 1, 1, 0.7]);

    fprintf('Animating  |  skip:%d  speed:%.1fx\n', cfg.skip_frames, cfg.anim_speed);

    % -- Animation loop --
    for step = 1:cfg.skip_frames:n_steps
        tic;
        t = time_steps(step);

        % -- Update each robot's distance line --
        for i = 1:num_robots
            set(robot_lines(i), 'XData', time_steps(1:step), ...
                                'YData', closest_distances(i, 1:step));
        end

        % -- Update timestamp --
        set(h_time, 'String', sprintf('t = %.2f s', t));
        drawnow;

        % -- Frame timing --
        elapsed = toc;
        pause_t = (dt * cfg.skip_frames) / cfg.anim_speed - elapsed;
        if pause_t > 0, pause(pause_t); end
        if cfg.save_anim, writeVideo(vid, getframe(fig2)); end

    end  % animation loop

    fprintf('Animation complete.\n');
    if cfg.save_anim
        close(vid);
        fprintf('  Video saved : %s\n', cfg.anim_filename);
    end
end

%% 5. Statistics

fprintf('\n--- Distance Statistics ---\n');

[sys_min_val, sys_min_step] = min(system_min);
sys_violations = sum(system_min <= 0);
fprintf('System : Min = %.4f m at t = %.3f s,  Violations = %d\n\n', ...
    sys_min_val, (sys_min_step - 1) * dt, sys_violations);

for i = 1:num_robots
    [min_d, min_step] = min(closest_distances(i, :));
    mean_d     = mean(closest_distances(i, :));
    violations = sum(closest_distances(i, :) <= 0);
    fprintf('R%d : Min = %.4f m at t = %.3f s,  Mean = %.4f m,  Violations = %d\n', ...
        i, min_d, (min_step - 1) * dt, mean_d, violations);
end

fprintf('\nDone.\n');

%% LOCAL FUNCTIONS

% Compute minimum distance from an ellipse perimeter to a circle perimeter
function dist = computeEllipseToCircleDistance(ex, ey, ephi, ea, eb, cx, cy, cr)
    n_samples = 72;
    angles    = linspace(0, 2*pi, n_samples);
    min_d     = inf;
    for ang = angles
        lx = ea * cos(ang);
        ly = eb * sin(ang);
        wx = ex + lx * cos(ephi) - ly * sin(ephi);
        wy = ey + lx * sin(ephi) + ly * cos(ephi);
        d  = sqrt((wx - cx)^2 + (wy - cy)^2) - cr;
        if d < min_d, min_d = d; end
    end
    dist = min_d;
end

% Compute minimum distance between two ellipse perimeters
function dist = computeEllipseToEllipseDistance(ex1, ey1, ephi1, ea1, eb1, ...
                                                 ex2, ey2, ephi2, ea2, eb2)
    n_samples = 72;
    angles    = linspace(0, 2*pi, n_samples);
    min_d     = inf;
    for ang = angles
        lx = ea1 * cos(ang);
        ly = eb1 * sin(ang);
        wx = ex1 + lx * cos(ephi1) - ly * sin(ephi1);
        wy = ey1 + lx * sin(ephi1) + ly * cos(ephi1);
        d  = computePointToEllipseDistance(wx, wy, ex2, ey2, ephi2, ea2, eb2);
        if d < min_d, min_d = d; end
    end
    dist = min_d;
end

% Compute minimum distance from a point to an ellipse perimeter
function dist = computePointToEllipseDistance(px, py, ex, ey, ephi, ea, eb)
    n_samples = 72;
    angles    = linspace(0, 2*pi, n_samples);
    min_d     = inf;
    for ang = angles
        lx = ea * cos(ang);
        ly = eb * sin(ang);
        wx = ex + lx * cos(ephi) - ly * sin(ephi);
        wy = ey + lx * sin(ephi) + ly * cos(ephi);
        d  = sqrt((wx - px)^2 + (wy - py)^2);
        if d < min_d, min_d = d; end
    end
    dist = min_d;
end