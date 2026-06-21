%% B_Plot_CBF_CR_Test.m
% Plot : 2-Robot CBF-Based Conflict Resolution (CBF-CR) Test
% Load : CBF_CR_Test.mat (run B_Data_Sim_CBF_CR_Test.m first)
% All display settings are in cfg at the top of this file.

clear; clc;

%% 0. Display Configuration

% -- Mode toggles --
cfg.run_plot = true;     % true = run trajectory plot
cfg.run_anim = false;     % true = run animation

% -- Save toggles --
cfg.save_plot     = false;                    % true = export PNG
cfg.save_anim     = false;                    % true = export MP4
cfg.plot_filename = 'CBF_CR_Plot.png';       % output plot filename
cfg.anim_filename = 'CBF_CR_Anim.mp4';       % output video filename

% -- Figure size [left, bottom, width, height], px --
cfg.plot_fig_pos = [50,  100, 1050, 720];
cfg.anim_fig_pos = [100,  80, 1050, 720];

% -- Font --
cfg.font_name        = 'Times New Roman';
cfg.font_size_title  = 30;
cfg.font_size_axis   = 24;
cfg.font_size_legend = 16;
cfg.font_size_label  = 14;

% -- Trajectory line and arrows --
cfg.traj_line_width = 2.4;
cfg.n_arrows        = 3;     % direction arrowheads per trajectory

% -- Animation playback --
cfg.skip_frames = 10;    % render every Nth step (10 -> ~10 fps at dt=0.01)
cfg.anim_speed  = 1.0;   % playback speed multiplier (1.0 = real-time)

%% 1. Load .mat File

mat_filename = 'CBF_CR_Test.mat';
fprintf('=== B_Plot_CBF_CR_Test.m ===\n');
if ~exist(mat_filename, 'file')
    error('Required file not found: %s\n  Run B_Data_Sim_CBF_CR_Test.m first.', mat_filename);
end
d = load(mat_filename);
fprintf('  Loaded : %s\n', mat_filename);

robot_params   = d.robot_params;
ctrl_params    = d.ctrl_params;
num_robots     = d.num_robots;       % 2
n_steps        = d.n_steps;
time_steps     = d.time_steps;
dt             = d.dt;
cbf_cr_enabled = d.cbf_cr_enabled;  % drives titles and labels

% -- Per-robot colors (lines palette) --
anim_clrs    = lines(max(num_robots, 4));
line_styles  = {'-', '-.'};
robot_labels = {'R$_1$', 'R$_2$'};

% -- Auto axis limits from trajectory extents --
all_x = [];  all_y = [];
for i = 1:num_robots
    traj  = d.trajectory_history{i}(1:n_steps, :);
    all_x = [all_x; traj(:,1)]; %#ok<AGROW>
    all_y = [all_y; traj(:,2)]; %#ok<AGROW>
end
pad         = 1.5;
axis_limits = [min(all_x)-pad, max(all_x)+pad, min(all_y)-pad, max(all_y)+pad];

% -- Titles driven by cbf_cr_enabled flag --
if cbf_cr_enabled
    plot_title = '(a) With CBF-CR';
    anim_title = 'Two Robots Conflict with CBF-CR';
else
    plot_title = '(b) Without CBF-CR';
    anim_title = 'Two Robots Conflict without CBF-CR';
end

%% 2. Trajectory Plot

if cfg.run_plot

    fig1 = figure('Name', 'CBF-CR Test - Trajectory Plot', ...
        'Position', cfg.plot_fig_pos, 'Color', 'w');
    ax1  = axes('Parent', fig1);
    hold(ax1, 'on');  grid(ax1, 'off');  axis(ax1, 'equal');  box(ax1, 'on');
    set(ax1, 'FontSize', cfg.font_size_axis, 'FontName', cfg.font_name, 'LineWidth', 1.0);
    xlabel(ax1, '$x$ Position (m)', 'FontSize', cfg.font_size_axis, 'Interpreter', 'latex');
    ylabel(ax1, '$y$ Position (m)', 'FontSize', cfg.font_size_axis, 'Interpreter', 'latex');
    title(ax1, plot_title, 'FontSize', cfg.font_size_title, ...
        'FontWeight', 'bold', 'FontName', cfg.font_name);
    xlim(ax1, axis_limits(1:2));
    ylim(ax1, axis_limits(3:4));

    % -- Trajectories and direction arrowheads --
    for i = 1:num_robots
        ac   = anim_clrs(i,:);
        traj = d.trajectory_history{i}(1:n_steps, :);
        ls   = line_styles{min(i, numel(line_styles))};

        % Trajectory line
        h = plot(ax1, traj(:,1), traj(:,2), ls, 'Color', ac, 'LineWidth', cfg.traj_line_width);
        set(h, 'DisplayName', robot_labels{i});

        % Direction arrowheads - solid triangle, no tail
        arr_idx = round(linspace(1, n_steps, cfg.n_arrows + 2));
        arr_idx = arr_idx(2:end-1);
        for k = arr_idx
            dx    = traj(k+1,1) - traj(k-1,1);
            dy    = traj(k+1,2) - traj(k-1,2);
            phi_k = atan2(dy, dx);
            tip   = 0.20;  base = 0.12;
            verts = [tip, 0;  0, base/2;  0, -base/2];
            R     = [cos(phi_k), -sin(phi_k); sin(phi_k), cos(phi_k)];
            rv    = (R * verts')';
            patch(ax1, rv(:,1)+traj(k,1), rv(:,2)+traj(k,2), ac, ...
                'EdgeColor', ac, 'LineWidth', 1, 'HandleVisibility', 'off');
        end
    end

    % -- Goal markers (drawn after trajectories to avoid legend duplication) --
    for i = 1:num_robots
        ac = anim_clrs(i,:);
        gp = d.robots(i).target_poses;
        plot(ax1, gp(1), gp(2), 'p', 'MarkerSize', 20, ...
            'MarkerFaceColor', ac, 'MarkerEdgeColor', 'k', 'LineWidth', 1.5, ...
            'HandleVisibility', 'off');
        text(ax1, gp(1), gp(2)+0.40, sprintf('R$_%d$ Goal', i), ...
            'FontSize', cfg.font_size_label, 'Color', ac, ...
            'FontName', cfg.font_name, 'FontWeight', 'bold', ...
            'Interpreter', 'latex', 'HorizontalAlignment', 'center');
    end

    legend(ax1, 'show', 'Location', 'northeast', 'FontSize', cfg.font_size_legend, ...
        'FontName', cfg.font_name, 'Interpreter', 'latex');
    hold off;
    fprintf('Trajectory plot complete.\n');

    if cfg.save_plot
        exportgraphics(fig1, cfg.plot_filename, 'Resolution', 300);
        fprintf('  Plot saved : %s\n', cfg.plot_filename);
    end
end

%% 3. Animation

if cfg.run_anim

    % -- Video setup --
    if cfg.save_anim
        vid           = VideoWriter(cfg.anim_filename, 'MPEG-4');
        vid.FrameRate = 10;
        open(vid);
        fprintf('  Recording : %s\n', cfg.anim_filename);
    end

    % -- Figure and axes setup --
    fig2 = figure('Name', 'CBF-CR Test - Animation', ...
        'Color', 'w', 'Position', cfg.anim_fig_pos);
    ax2  = axes('Parent', fig2);
    hold(ax2, 'on');  grid(ax2, 'off');  axis(ax2, 'equal');  box(ax2, 'on');
    set(ax2, 'FontSize', cfg.font_size_axis, 'FontName', cfg.font_name, 'LineWidth', 1.0);
    xlabel(ax2, '$x$ Position (m)', 'FontSize', cfg.font_size_axis, 'Interpreter', 'latex');
    ylabel(ax2, '$y$ Position (m)', 'FontSize', cfg.font_size_axis, 'Interpreter', 'latex');
    title(ax2, anim_title, 'FontSize', cfg.font_size_title, ...
        'FontWeight', 'bold', 'FontName', cfg.font_name);
    xlim(ax2, axis_limits(1:2));
    ylim(ax2, axis_limits(3:4));

    % -- Static goal markers --
    for i = 1:num_robots
        ac = anim_clrs(i,:);
        gp = d.robots(i).target_poses;
        plot(ax2, gp(1), gp(2), 'p', 'MarkerSize', 20, ...
            'MarkerFaceColor', ac, 'MarkerEdgeColor', 'k', 'LineWidth', 1.5, ...
            'HandleVisibility', 'off');
        text(ax2, gp(1), gp(2)+0.40, sprintf('R$_%d$ Goal', i), ...
            'FontSize', cfg.font_size_label, 'Color', ac, ...
            'FontName', cfg.font_name, 'FontWeight', 'bold', ...
            'Interpreter', 'latex', 'HorizontalAlignment', 'center');
    end

    % -- Pre-allocate animated graphic objects --
    n_ep    = 50;
    r_dot   = 0.10;
    phi_dot = linspace(0, 2*pi, 20);

    h_ctr = gobjects(num_robots, 1);  % centre dot
    h_bod = gobjects(num_robots, 1);  % ellipse body
    h_hd  = gobjects(num_robots, 1);  % heading arrow
    h_lbl = gobjects(num_robots, 1);  % robot label
    h_trl = gobjects(num_robots, 1);  % motion trail

    for i = 1:num_robots
        ac       = anim_clrs(i,:);
        h_ctr(i) = patch(ax2, 0, 0, ac, 'EdgeColor', 'none', 'FaceAlpha', 1.0);
        h_bod(i) = patch(ax2, zeros(1,n_ep), zeros(1,n_ep), ac, ...
            'FaceAlpha', 0.6, 'EdgeColor', ac, 'LineWidth', 2);
        h_hd(i)  = patch(ax2, [0,0,0], [0,0,0], ac, ...
            'EdgeColor', 'k', 'LineWidth', 1.5, 'FaceAlpha', 0.9);
        h_lbl(i) = text(ax2, 0, 0, sprintf('R_{%d}', i), ...
            'HorizontalAlignment', 'center', 'FontSize', 11, ...
            'FontWeight', 'bold', 'Color', 'w', 'FontName', cfg.font_name);
        h_trl(i) = plot(ax2, NaN, NaN, '-', 'Color', [ac, 0.45], 'LineWidth', 2.5);
    end

    h_time = text(ax2, 0.02, 0.97, '', 'Units', 'normalized', ...
        'FontSize', 16, 'FontWeight', 'bold', 'VerticalAlignment', 'top', ...
        'FontName', cfg.font_name, 'BackgroundColor', [1, 1, 1, 0.7]);

    fprintf('Animating  |  skip:%d  speed:%.1fx\n', cfg.skip_frames, cfg.anim_speed);

    % -- Animation loop --
    for step = 1:cfg.skip_frames:n_steps
        tic;
        t = time_steps(step);

        for i = 1:num_robots
            pose = d.trajectory_history{i}(step, :);
            xr   = pose(1);  yr = pose(2);  phi = pose(3);

            % -- Update robot body, heading, label, and trail --
            set(h_ctr(i), 'XData', xr + r_dot*cos(phi_dot), ...
                          'YData', yr + r_dot*sin(phi_dot));
            [ex, ey] = rotatedEllipse(xr, yr, phi, robot_params.length, robot_params.width, n_ep);
            set(h_bod(i), 'XData', ex, 'YData', ey);
            [ghx, ghy] = arrowHead(xr, yr, phi, robot_params.length, robot_params.width);
            set(h_hd(i),  'XData', ghx, 'YData', ghy);
            set(h_lbl(i), 'Position', [xr, yr]);
            ts = max(1, step - 200);
            td = d.trajectory_history{i}(ts:step, :);
            set(h_trl(i), 'XData', td(:,1), 'YData', td(:,2));
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

%% LOCAL FUNCTIONS

% Compute rotated ellipse vertices for robot body rendering
function [xe, ye] = rotatedEllipse(xc, yc, phi, len, wid, np)
    a  = len/2;  b = wid/2;
    tt = linspace(0, 2*pi, np);
    R  = [cos(phi), -sin(phi); sin(phi), cos(phi)];
    g  = R * [a*cos(tt); b*sin(tt)];
    xe = g(1,:) + xc;
    ye = g(2,:) + yc;
end

% Compute heading arrowhead vertices for robot orientation rendering
function [ghx, ghy] = arrowHead(xr, yr, phi, rlen, rwid)
    hl = rlen * 0.15;
    hw = rwid * 0.35;
    lh = [rlen/2 + hl,  0;
          rlen/2,        hw/2;
          rlen/2,       -hw/2];
    R  = [cos(phi), -sin(phi); sin(phi), cos(phi)];
    gh = (R * lh')';
    ghx = gh(:,1) + xr;
    ghy = gh(:,2) + yr;
end