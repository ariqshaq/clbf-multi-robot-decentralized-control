%% A_Plot_Multi_Robot.m
% Plot : Multi-Robot Warehouse Navigation, Snapshot / Animation
% Load : sim_results.mat (run A_Data_Sim_Multi_Robot.m first)
% All display settings are in cfg at the top of this file.

clear; clc;

%% 0. Display Configuration

% -- Mode --
% 1 -> Main Video
% 2 -> Detail Video (w/ CBF margin & sub-target)
% 3 -> LiDAR Video (one robot perspective w/ LiDAR points)
% 4 -> Snapshot (image only)
cfg.mode = 2;

% -- Snapshot time (seconds) --
cfg.snapshot_time = 146.30;

% -- Save toggles --
cfg.save_image = false;                       % true = export PNG (mode 4)
cfg.save_video = false;                       % true = export MP4 (modes 1-3)
cfg.image_filename = '';                      % '' -> auto-name: Snapshot_t150s.png
cfg.video_filename = 'Multi_Robot_Animation.mp4';   % overridden below per mode

% -- Figure --
cfg.fullscreen_mode = false;
cfg.fig_pos = [100, 100, 1000, 750];   % [left, bottom, width, height], px

% -- Font --
cfg.font_name        = 'Times New Roman';
cfg.font_size_title  = 30;
cfg.font_size_axis   = 24;
cfg.font_size_label  = 12;   % robot ID label inside body

% -- Title (set '' to auto-generate) --
cfg.plot_title = '';

% -- Trail --
cfg.trail_length     = 100;   % past steps shown per robot/human
cfg.trail_line_width = 2.5;

% -- Task star marker (waypoint target; modes 1, 3, 4) --
cfg.star_size = 12;

% -- Safety margin detection range (mode 2 only) --
cfg.margin_detect_range = 2.0;   % m; ring shown when a neighbour is within this distance

% -- Playback (modes 1-3 only) --
cfg.skip_frames     = 10;     % render every Nth step (1 = no skip)
cfg.animation_speed = 200.0;  % speed multiplier (200 -> ~1:1 time ratio at t_max=200s)
cfg.video_framerate = 10;     % frames per second in output .mp4

%% 1. Load .mat File

mat_filename = 'sim_results.mat';
fprintf('=== A_Plot_Multi_Robot.m ===\n');
if ~exist(mat_filename, 'file')
    error('Required file not found: %s\n  Run A_Data_Sim_Multi_Robot.m first.', mat_filename);
end
d = load(mat_filename);
fprintf('  Loaded : %s\n', mat_filename);

trajectory_history       = d.trajectory_history;
task_history              = d.task_history;
subtarget_history         = d.subtarget_history;
human_trajectory_history  = d.human_trajectory_history;
saved_sensor_history      = d.saved_sensor_history;
saved_sensor_robot        = d.saved_sensor_robot;
time_steps                = d.time_steps(1:d.n_steps);
n_steps                   = d.n_steps;
num_robots                = d.num_robots;
num_humans                = d.num_humans;
dt                        = d.dt;
warehouse                 = d.warehouse;
docks                     = d.docks;
zones                     = d.zones;
lidar_circle_obs          = d.lidar_circle_obs;
robot_params              = d.robot_params;
ctrl_params               = d.ctrl_params;
lidar_params              = d.lidar_params;
initial_poses             = d.initial_poses;
humans                    = d.humans;

fprintf('  Robots : %d  |  Humans : %d  |  Steps : %d  (%.1f s)\n', ...
    num_robots, num_humans, n_steps, time_steps(end));

% -- LiDAR variant always centers on the robot whose sensor history was saved --
lidar_robot = saved_sensor_robot;

% -- Pre-allocation bound for LiDAR margin-ring handles (mode 3) --
cfg.max_lidar_detections = lidar_params.num_samples;

%% 2. Shared Display Setup

% -- Shared colors --
colors      = lines(num_robots);
human_color = [1, 0.5, 0];
theta_circ  = linspace(0, 2*pi, 64);

%% 3. Snapshot Mode (Mode 4)

if cfg.mode == 4

    [~, step] = min(abs(time_steps - cfg.snapshot_time));
    t_snap    = time_steps(step);
    fprintf('  Snapshot : requested %.2f s -> closest step %d (t = %.4f s)\n\n', ...
        cfg.snapshot_time, step, t_snap);

    fig = figure('Name', 'Multi-Robot Snapshot', 'Color', 'w', 'Position', cfg.fig_pos);
    ax  = axes('Parent', fig);
    hold(ax, 'on'); grid(ax, 'off'); axis(ax, 'equal');
    set(ax, 'FontSize', cfg.font_size_axis, 'FontName', cfg.font_name, 'LineWidth', 1.0);
    xlabel(ax, '$x$ Position (m)', 'FontSize', cfg.font_size_axis, 'Interpreter', 'latex');
    ylabel(ax, '$y$ Position (m)', 'FontSize', cfg.font_size_axis, 'Interpreter', 'latex');
    if ~isempty(cfg.plot_title)
        title(ax, cfg.plot_title, 'FontSize', cfg.font_size_title, ...
            'FontName', cfg.font_name, 'FontWeight', 'bold', 'Interpreter', 'latex');
    else
        title(ax, sprintf('t = %.2f s', t_snap), 'FontSize', cfg.font_size_title, ...
            'FontName', cfg.font_name, 'FontWeight', 'bold');
    end
    xlim(ax, [-0.5, warehouse.x_size+0.5]);
    ylim(ax, [-0.5, warehouse.y_size+0.5]);

    drawStaticEnvironment(ax, warehouse, docks, zones, lidar_circle_obs, ...
        initial_poses, colors, num_robots, true);

    % -- Humans at snapshot step --
    for h = 1:num_humans
        hp = human_trajectory_history{h}(step,:);
        ts = max(1, step - cfg.trail_length);
        trail = human_trajectory_history{h}(ts:step,:);
        plot(ax, trail(:,1), trail(:,2), ':', 'Color', [human_color, 0.6], ...
            'LineWidth', 2.0, 'HandleVisibility', 'off');
        fill(ax, hp(1) + humans(h).radius*cos(theta_circ), ...
                 hp(2) + humans(h).radius*sin(theta_circ), ...
            human_color, 'FaceAlpha', 0.7, 'EdgeColor', 'k', 'LineWidth', 2, ...
            'HandleVisibility', 'off');
        text(ax, hp(1), hp(2), 'H', 'HorizontalAlignment', 'center', ...
            'VerticalAlignment', 'middle', 'FontSize', 10, 'FontWeight', 'bold', 'Color', 'k');
    end

    % -- Robots at snapshot step --
    n_ep  = 50;
    r_dot = 0.1;
    phi_d = linspace(0, 2*pi, 20);

    for i = 1:num_robots
        cl   = colors(i,:);
        pose = trajectory_history{i}(step,:);
        xr = pose(1); yr = pose(2); th = pose(3);

        ts = max(1, step - cfg.trail_length);
        td = trajectory_history{i}(ts:step,:);
        plot(ax, td(:,1), td(:,2), '-', 'Color', [cl, 0.5], ...
            'LineWidth', cfg.trail_line_width, 'HandleVisibility', 'off');

        tgt = task_history{i}(step,:);
        if step > 1 && ~isequal(tgt, initial_poses(i,1:2)) && ~isequal(tgt, [0, 0])
            plot(ax, tgt(1), tgt(2), 'p', 'MarkerSize', cfg.star_size, ...
                'MarkerFaceColor', cl, 'MarkerEdgeColor', 'k', 'LineWidth', 1.5, ...
                'HandleVisibility', 'off');
        end

        [ex, ey] = rotatedEllipse(xr, yr, th, robot_params.length, robot_params.width, n_ep);
        patch(ax, ex, ey, cl, 'FaceAlpha', 0.6, 'EdgeColor', cl, 'LineWidth', 2, ...
            'HandleVisibility', 'off');

        hl = robot_params.length * 0.15;
        hw = robot_params.width  * 0.35;
        lh = [robot_params.length/2 + hl,  0;
              robot_params.length/2,        hw/2;
              robot_params.length/2,       -hw/2];
        Rot = [cos(th), -sin(th); sin(th), cos(th)];
        gh  = (Rot * lh')';
        patch(ax, gh(:,1)+xr, gh(:,2)+yr, cl, 'EdgeColor', 'k', 'LineWidth', 1.5, ...
            'FaceAlpha', 0.9, 'HandleVisibility', 'off');

        patch(ax, xr + r_dot*cos(phi_d), yr + r_dot*sin(phi_d), cl, ...
            'EdgeColor', 'none', 'FaceAlpha', 1.0, 'HandleVisibility', 'off');

        text(ax, xr, yr, sprintf('R_%d', i), 'HorizontalAlignment', 'center', ...
            'VerticalAlignment', 'middle', 'FontSize', cfg.font_size_label, ...
            'FontWeight', 'bold', 'Color', 'w');
    end

    drawnow;
    fprintf('Snapshot complete.\n');

    if cfg.save_image
        img_file = cfg.image_filename;
        if isempty(img_file)
            img_file = sprintf('Snapshot_t%.0fs.png', cfg.snapshot_time);
        end
        exportgraphics(fig, img_file, 'Resolution', 300);
        fprintf('  Image saved : %s\n', img_file);
    end
end

%% 4. Video Mode (Mode 1, 2, or 3)

if cfg.mode == 1 || cfg.mode == 2 || cfg.mode == 3

    % -- Resolve mode-specific behaviour --
    lidar_mode    = (cfg.mode == 3);
    details_mode  = (cfg.mode == 2);
    show_bounds   = ~lidar_mode;    % mode 3 hides warehouse boundary & statics
    show_humans   = ~lidar_mode;    % mode 3 hides humans
    show_trail    = ~lidar_mode;    % mode 3 hides motion trail

    if lidar_mode
        anim_title = sprintf('Robot %d LiDAR Points', lidar_robot);
        if strcmp(cfg.video_filename, 'Multi_Robot_Animation.mp4')
            cfg.video_filename = 'Multi_Robot_Animation_w_LiDAR.mp4';
        end
    elseif details_mode
        anim_title = 'Multi-Robot System Details';
        if strcmp(cfg.video_filename, 'Multi_Robot_Animation.mp4')
            cfg.video_filename = 'Multi_Robot_Animation_w_Details.mp4';
        end
    else
        anim_title = 'Decentralized Multi-Robot System';
    end
    if ~isempty(cfg.plot_title)
        anim_title = cfg.plot_title;
    end

    % -- Video setup --
    if cfg.save_video
        vid           = VideoWriter(cfg.video_filename, 'MPEG-4');
        vid.FrameRate = cfg.video_framerate;
        open(vid);
        fprintf('  Recording : %s\n', cfg.video_filename);
    end

    % -- Figure and axes setup --
    fig = figure('Name', 'Multi-Robot Navigation Animation', 'Color', 'w');
    if cfg.fullscreen_mode
        set(fig, 'Units', 'normalized', 'Position', [0, 0, 1, 1]);
    else
        set(fig, 'Position', cfg.fig_pos);
    end
    ax = axes('Parent', fig);
    hold(ax, 'on'); grid(ax, 'off'); axis(ax, 'equal');
    set(ax, 'FontSize', cfg.font_size_axis, 'FontName', cfg.font_name, 'LineWidth', 1.0);
    xlabel(ax, '$x$ Position (m)', 'FontSize', cfg.font_size_axis, 'Interpreter', 'latex');
    ylabel(ax, '$y$ Position (m)', 'FontSize', cfg.font_size_axis, 'Interpreter', 'latex');
    title(ax, anim_title, 'FontSize', cfg.font_size_title, ...
        'FontName', cfg.font_name, 'FontWeight', 'bold');
    xlim(ax, [-0.5, warehouse.x_size+0.5]);
    ylim(ax, [-0.5, warehouse.y_size+0.5]);

    drawStaticEnvironment(ax, warehouse, docks, zones, lidar_circle_obs, ...
        initial_poses, colors, num_robots, show_bounds);

    % -- Graphics object pre-allocation --
    n_ep  = 50;
    r_dot = 0.1;
    phi_d = linspace(0, 2*pi, 20);

    robot_center  = gobjects(num_robots, 1);
    robot_bodies  = gobjects(num_robots, 1);
    robot_heads   = gobjects(num_robots, 1);
    robot_labels  = gobjects(num_robots, 1);
    robot_trails  = gobjects(num_robots, 1);
    robot_subtgt  = gobjects(num_robots, 1);   % dot marker, mode 2 (sub-target)
    robot_star    = gobjects(num_robots, 1);   % star marker, modes 1 & 3 (waypoint target)
    robot_margin  = gobjects(num_robots, 1);   % CBF-CR margin, mode 2 only

    for i = 1:num_robots
        cl = colors(i,:);
        robot_center(i) = patch(ax, 0, 0, cl, 'EdgeColor', 'none', 'FaceAlpha', 1.0);
        robot_bodies(i) = patch(ax, zeros(1,n_ep), zeros(1,n_ep), cl, ...
            'FaceAlpha', 0.6, 'EdgeColor', cl, 'LineWidth', 2);
        robot_heads(i)  = patch(ax, [0,0,0], [0,0,0], cl, ...
            'EdgeColor', 'k', 'LineWidth', 1.5, 'FaceAlpha', 0.9);
        robot_labels(i) = text(ax, 0, 0, sprintf('R_%d', i), ...
            'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
            'FontSize', 12, 'FontWeight', 'bold', 'Color', 'w');
        robot_trails(i) = plot(ax, NaN, NaN, '-', 'Color', [cl, 0.5], 'LineWidth', 2.5);
        robot_subtgt(i) = patch(ax, 0, 0, cl, 'EdgeColor', 'k', 'LineWidth', 1.5, 'FaceAlpha', 1.0);
        robot_star(i)   = plot(ax, NaN, NaN, 'p', 'MarkerSize', cfg.star_size, ...
            'MarkerFaceColor', cl, 'MarkerEdgeColor', 'k', 'LineWidth', 1.5);
        robot_margin(i) = plot(ax, NaN, NaN, '--', 'Color', [cl, 0.6], 'LineWidth', 1.5);
    end

    % -- Human graphics handles --
    human_bodies = gobjects(num_humans, 1);
    human_labels = gobjects(num_humans, 1);
    human_trails = gobjects(num_humans, 1);
    human_margin = gobjects(num_humans, 1);   % CBF margin, mode 2 only

    for h = 1:num_humans
        human_bodies(h) = patch(ax, zeros(1,n_ep), zeros(1,n_ep), human_color, ...
            'FaceAlpha', 0.7, 'EdgeColor', 'k', 'LineWidth', 2);
        human_labels(h) = text(ax, NaN, NaN, 'H', 'HorizontalAlignment', 'center', ...
            'VerticalAlignment', 'middle', 'FontSize', 10, 'FontWeight', 'bold', 'Color', 'k');
        human_trails(h) = plot(ax, NaN, NaN, ':', 'Color', [human_color, 0.6], 'LineWidth', 2.5);
        human_margin(h) = plot(ax, NaN, NaN, '--', 'Color', [human_color, 0.6], 'LineWidth', 1.5);
    end

    % -- Static circle obstacle margin handles (mode 2 only) --
    static_margin = gobjects(size(lidar_circle_obs, 1), 1);
    for k = 1:size(lidar_circle_obs, 1)
        static_margin(k) = plot(ax, NaN, NaN, '--', 'Color', [0.3, 0.3, 0.3, 0.6], 'LineWidth', 1.5);
    end

    % -- LiDAR handles (one robot only, lidar_robot) --
    lidar_pts_scatter = scatter(ax, NaN, NaN, 25, colors(lidar_robot,:), 'filled', ...
        'MarkerEdgeColor', 'none', 'LineWidth', 0.8, 'MarkerFaceAlpha', 0.8);
    lidar_margin = gobjects(cfg.max_lidar_detections, 1);
    for L = 1:cfg.max_lidar_detections
        lidar_margin(L) = plot(ax, NaN, NaN, '--', ...
            'Color', [colors(lidar_robot,:), 0.4], 'LineWidth', 1.0);
    end

    % -- Time display --
    time_text = text(ax, 0.03, 1.0, '', 'Units', 'normalized', ...
        'FontSize', 16, 'FontWeight', 'bold', 'VerticalAlignment', 'top', ...
        'FontName', cfg.font_name, 'BackgroundColor', [1, 1, 1, 0.7]);

    fprintf('Animating  |  skip:%d  speed:%.0fx\n', cfg.skip_frames, cfg.animation_speed);

    % -- Static circle obstacle margin rings drawn once (mode 2 only; obstacle pose is fixed) --
    if details_mode
        for k = 1:size(lidar_circle_obs, 1)
            r_static = lidar_circle_obs(k,3) + ctrl_params.mu_obstacle;
            set(static_margin(k), ...
                'XData', lidar_circle_obs(k,1) + r_static*cos(theta_circ), ...
                'YData', lidar_circle_obs(k,2) + r_static*sin(theta_circ));
        end
    end

    % -- Animation loop --
    for step = 1:cfg.skip_frames:n_steps
        tic;
        t = time_steps(step);

        current_positions = zeros(num_robots, 2);
        for i = 1:num_robots
            current_positions(i,:) = trajectory_history{i}(step, 1:2);
        end

        % -- Update humans --
        if show_humans
            for h = 1:num_humans
                hp = human_trajectory_history{h}(step,:);
                xc = hp(1) + humans(h).radius*cos(theta_circ);
                yc = hp(2) + humans(h).radius*sin(theta_circ);
                set(human_bodies(h), 'XData', xc, 'YData', yc);
                set(human_labels(h), 'Position', [hp(1), hp(2)]);

                ts = max(1, step - cfg.trail_length);
                trail = human_trajectory_history{h}(ts:step,:);
                set(human_trails(h), 'XData', trail(:,1), 'YData', trail(:,2));

                % -- CBF margin around human (mode 2 only) --
                if details_mode
                    r_in = humans(h).radius + ctrl_params.mu_obstacle;
                    set(human_margin(h), ...
                        'XData', hp(1) + r_in*cos(theta_circ), ...
                        'YData', hp(2) + r_in*sin(theta_circ));
                end
            end
        end

        % -- Update robots --
        robot_loop_range = 1:num_robots;
        if lidar_mode
            robot_loop_range = lidar_robot;   % one robot perspective
        end

        for i = robot_loop_range
            pose = trajectory_history{i}(step,:);
            x = pose(1); y = pose(2); phi = pose(3);
            cl = colors(i,:);

            set(robot_center(i), 'XData', x + r_dot*cos(phi_d), 'YData', y + r_dot*sin(phi_d));

            [ex, ey] = rotatedEllipse(x, y, phi, robot_params.length, robot_params.width, n_ep);
            set(robot_bodies(i), 'XData', ex, 'YData', ey);

            hl = robot_params.length * 0.1;
            hw = robot_params.width  * 0.3;
            lh = [robot_params.length/2 + hl,  0;
                  robot_params.length/2,        hw/2;
                  robot_params.length/2,       -hw/2];
            Rot = [cos(phi), -sin(phi); sin(phi), cos(phi)];
            gh  = (Rot * lh')';
            set(robot_heads(i), 'XData', gh(:,1)+x, 'YData', gh(:,2)+y);

            set(robot_labels(i), 'Position', [x, y]);

            if show_trail
                ts = max(1, step - cfg.trail_length);
                td = trajectory_history{i}(ts:step,:);
                set(robot_trails(i), 'XData', td(:,1), 'YData', td(:,2));
            end

            % -- CBF-CR margin around robot i (mode 2 only) --
            if details_mode
                mu_ring  = ctrl_params.mu_obstacle;   % default: no neighbor in range
                near     = false;
                for j = 1:num_robots
                    if i == j, continue; end
                    if norm(current_positions(i,:) - current_positions(j,:)) < cfg.margin_detect_range
                        % -- Margin j applies toward i, from j's perspective --
                        if j > i
                            mu_from_j = ctrl_params.mu_robot;     % j yields to i -> larger margin
                        else
                            mu_from_j = ctrl_params.mu_obstacle;  % j has priority -> standard margin
                        end
                        if ~near
                            mu_ring = mu_from_j;
                        else
                            mu_ring = min(mu_ring, mu_from_j);
                        end
                        near = true;
                    end
                end
                if near
                    r_ring = mu_ring + robot_params.length/2;
                    set(robot_margin(i), ...
                        'XData', x + r_ring*cos(theta_circ), ...
                        'YData', y + r_ring*sin(theta_circ));
                else
                    set(robot_margin(i), 'XData', NaN, 'YData', NaN);
                end
            end

            % -- Sub-target marker --
            if details_mode
                % Mode 2: path prediction sub-target dot (subtarget_history)
                set(robot_star(i), 'XData', NaN, 'YData', NaN);
                subtgt = subtarget_history{i}(step,:);
                if step == 1 || isequal(subtgt, initial_poses(i,1:2)) || isequal(subtgt, [0, 0])
                    set(robot_subtgt(i), 'XData', NaN, 'YData', NaN);
                else
                    set(robot_subtgt(i), ...
                        'XData', subtgt(1) + (r_dot*0.8)*cos(phi_d), ...
                        'YData', subtgt(2) + (r_dot*0.8)*sin(phi_d));
                end
            else
                % Modes 1 and 3: waypoint star marker (task_history)
                set(robot_subtgt(i), 'XData', NaN, 'YData', NaN);
                tgt = task_history{i}(step,:);
                if step == 1 || isequal(tgt, initial_poses(i,1:2)) || isequal(tgt, [0, 0])
                    set(robot_star(i), 'XData', NaN, 'YData', NaN);
                else
                    set(robot_star(i), 'XData', tgt(1), 'YData', tgt(2));
                end
            end

            % -- LiDAR detection points (mode 3 only, lidar_robot) --
            if lidar_mode && i == lidar_robot
                if ~isempty(saved_sensor_history{step})
                    lidar_det = saved_sensor_history{step};
                    n_det     = size(lidar_det, 1);
                    set(lidar_pts_scatter, 'XData', lidar_det(:,1), 'YData', lidar_det(:,2));

                    n_show = min(n_det, cfg.max_lidar_detections);
                    for L = 1:n_show
                        r_in = ctrl_params.mu_obstacle;
                        set(lidar_margin(L), ...
                            'XData', lidar_det(L,1) + r_in*cos(theta_circ), ...
                            'YData', lidar_det(L,2) + r_in*sin(theta_circ));
                    end
                    for L = n_show+1:cfg.max_lidar_detections
                        set(lidar_margin(L), 'XData', NaN, 'YData', NaN);
                    end
                else
                    set(lidar_pts_scatter, 'XData', NaN, 'YData', NaN);
                    for L = 1:cfg.max_lidar_detections
                        set(lidar_margin(L), 'XData', NaN, 'YData', NaN);
                    end
                end
            end

        end  % robot loop

        % -- Update timestamp --
        set(time_text, 'String', sprintf('t = %.2f s', t));
        drawnow;

        % -- Frame timing --
        elapsed = toc;
        pause_t = (dt * cfg.skip_frames) / cfg.animation_speed - elapsed;
        if pause_t > 0, pause(pause_t); end

        if mod(step, 500) == 0
            fprintf('  %.1f %%   t = %.1f s\n', step/n_steps*100, t);
        end

        if cfg.save_video, writeVideo(vid, getframe(fig)); end

    end  % animation loop

    fprintf('Animation complete.\n');
    if cfg.save_video
        close(vid);
        fprintf('  Video saved : %s\n', cfg.video_filename);
    end
end

%% LOCAL FUNCTIONS

% Draw shared static environment: warehouse bounds, docks, zones, statics, parking spots
function drawStaticEnvironment(ax, warehouse, docks, zones, lidar_circle_obs, ...
        initial_poses, colors, num_robots, show_bounds)
    % -- Warehouse boundary --
    if show_bounds
        rectangle(ax, 'Position', [0, 0, warehouse.x_size, warehouse.y_size], ...
            'EdgeColor', 'k', 'LineWidth', 2.0);
    end

    % -- Docks (filled rectangles + labels) --
    for k = 1:size(docks, 1)
        dxk = docks(k,1) - docks(k,3)/2;
        dyk = docks(k,2) - docks(k,4)/2;
        rectangle(ax, 'Position', [dxk, dyk, docks(k,3), docks(k,4)], ...
            'FaceColor', [0.8, 0.8, 0.8], 'EdgeColor', [0.3, 0.3, 0.3], 'LineWidth', 1.5);
        text(ax, docks(k,1), docks(k,2), sprintf('D_%d', k), ...
            'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'FontSize', 12);
    end

    % -- Dock zones (dashed outlines) --
    for k = 1:size(zones, 1)
        zxk = zones(k,1) - zones(k,3)/2;
        zyk = zones(k,2) - zones(k,4)/2;
        rectangle(ax, 'Position', [zxk, zyk, zones(k,3), zones(k,4)], ...
            'FaceColor', 'none', 'EdgeColor', [0.3, 0.3, 0.3], ...
            'LineWidth', 1.5, 'LineStyle', '--');
    end

    % -- Static circle obstacles --
    if show_bounds
        theta_c = linspace(0, 2*pi, 64);
        for k = 1:size(lidar_circle_obs, 1)
            ox = lidar_circle_obs(k,1);
            oy = lidar_circle_obs(k,2);
            orad = lidar_circle_obs(k,3);
            fill(ax, ox + orad*cos(theta_c), oy + orad*sin(theta_c), ...
                [0.3, 0.3, 0.3], 'EdgeColor', 'k', 'LineWidth', 1.5, ...
                'HandleVisibility', 'off');
        end
    end

    % -- Initial positions (faded dots) --
    if show_bounds
        for i = 1:num_robots
            plot(ax, initial_poses(i,1), initial_poses(i,2), 'o', ...
                'MarkerSize', 12, 'MarkerFaceColor', colors(i,:)*0.3, ...
                'MarkerEdgeColor', colors(i,:), 'LineWidth', 2, ...
                'HandleVisibility', 'off');
        end
    end
end

% Compute rotated ellipse vertices for robot body rendering
function [xe, ye] = rotatedEllipse(xc, yc, phi, len, wid, np)
    a  = len/2;  b = wid/2;
    tt = linspace(0, 2*pi, np);
    R  = [cos(phi), -sin(phi); sin(phi), cos(phi)];
    g  = R * [a*cos(tt); b*sin(tt)];
    xe = g(1,:) + xc;
    ye = g(2,:) + yc;
end