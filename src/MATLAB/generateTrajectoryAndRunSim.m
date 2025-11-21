function generateTrajectoryAndRunSim(path, libraryMap, mapInflated)
% =========================================================================
% GENERATE_TRAJECTORY_AND_RUN_SIM
% -------------------------------------------------------------------------
% This function takes a set of waypoints and generates a smooth,
% time-based trajectory using cubic polynomial interpolation.
% It then runs the 'diff_drive.slx' simulation and plots all
% desired vs. actual results.
%
% Inputs:
%   path         - [Nx2] matrix of waypoints from the path planner.
%   libraryMap   - The original, uninflated map object (for plotting).
%   mapInflated  - The inflated map object (unused here, but good practice).
% =========================================================================

%% ===== 1. Parameters for Trajectory Generation =====
% Shift waypoints to start from (0,0) for the trajectory generator
waypoints = path; % - path(1,:) .* ones(size(path));

% --- Timing ---
Tt = 50;                                % TOTAL DURATION in seconds
Ts = 0.05;                              % Sample time for the simulation
% --- Segment Calculation ---
n_segments = size(waypoints, 1) - 1;    % Number of segments
if n_segments == 0
    disp('Path has only one point. Cannot generate trajectory.');
    return;
end
T_segment = Tt / n_segments;            % Duration of each single segment
n_points_segment_approx = 50;           % Approx number of points per segment
n_points_total = n_points_segment_approx * n_segments;
n_points_segment = n_points_segment_approx; % Use the approx value

% --- Pre-allocate arrays ---
tt = zeros(n_points_total, 1);
path_x = zeros(n_points_total, 1);
path_y = zeros(n_points_total, 1);
path_theta = zeros(n_points_total, 1);
vel_x = zeros(n_points_total, 1);
vel_y = zeros(n_points_total, 1);
simulink_path_data = zeros(n_points_total, 4); % [time, x, y, theta]

%% ===== 2. Path Generation Loop =====
% This loop calculates the trajectory for each segment
for i = 1:n_segments
    % --- Time vectors ---
    t = linspace(0, T_segment, n_points_segment)'; % Local time (0 to T_segment)
    tx = linspace(T_segment*(i-1), T_segment*i, n_points_segment)'; % Global time
    
    % --- Boundary Conditions for Segment 'i' ---
    % Position
    xi = waypoints(i, 1);       % Initial X
    xf = waypoints(i+1, 1);     % Final X
    yi = waypoints(i, 2);       % Initial Y
    yf = waypoints(i+1, 2);     % Final Y
    % Velocity (0 at segment ends for stop-and-go)
    dxi = 0; dxf = 0;
    dyi = 0; dyf = 0;
    
    % --- System for X-coordinate coefficients ---
    % x(t) = a0 + a1*t + a2*t^2 + a3*t^3
    a0_x = xi;
    a1_x = dxi;
    A = [ T_segment^2    T_segment^3 ;
          2*T_segment    3*T_segment^2 ];
    B_x = [ xf - (a0_x + a1_x*T_segment) ;
            dxf - a1_x ];
    coef_x = A \ B_x;
    a2_x = coef_x(1);
    a3_x = coef_x(2);
    
    % --- System for Y-coordinate coefficients ---
    % y(t) = a0 + a1*t + a2*t^2 + a3*t^3
    a0_y = yi;
    a1_y = dyi;
    B_y = [ yf - (a0_y + a1_y*T_segment) ;
            dyf - a1_y ];
    coef_y = A \ B_y;
    a2_y = coef_y(1);
    a3_y = coef_y(2);
    
    % --- Calculate Path for this Segment ---
    q_x  = a0_x + a1_x*t + a2_x*t.^2 + a3_x*t.^3;
    q_y  = a0_y + a1_y*t + a2_y*t.^2 + a3_y*t.^3;
    dq_x = a1_x + 2*a2_x*t + 3*a3_x*t.^2;
    dq_y = a1_y + 2*a2_y*t + 3*a3_y*t.^2;
    q_theta = atan2(dq_y, dq_x);
    
    % --- Store segment data in global arrays ---
    idx_start = n_points_segment*(i-1) + 1;
    idx_end   = n_points_segment*i;
    
    % Handle last segment index mismatch
    if i == n_segments
        idx_end = n_points_total;
        % Recalculate last segment with correct number of points
        num_points_last = n_points_total - idx_start + 1;
        tx = linspace(T_segment*(i-1), Tt, num_points_last)';
        t = linspace(0, T_segment, num_points_last)';
        
        q_x  = a0_x + a1_x*t + a2_x*t.^2 + a3_x*t.^3;
        q_y  = a0_y + a1_y*t + a2_y*t.^2 + a3_y*t.^3;
        dq_x = a1_x + 2*a2_x*t + 3*a3_x*t.^2;
        dq_y = a1_y + 2*a2_y*t + 3*a3_y*t.^2;
        q_theta = atan2(dq_y, dq_x);
    end
    
    tt(idx_start:idx_end) = tx;
    path_x(idx_start:idx_end) = q_x;
    path_y(idx_start:idx_end) = q_y;
    path_theta(idx_start:idx_end) = q_theta;
    vel_x(idx_start:idx_end) = dq_x;
    vel_y(idx_start:idx_end) = dq_y;
    
    % --- Handle Theta discontinuity at stops ---
    if i > 1
        start_index = n_points_segment*(i-1) + 1;
        last_theta = path_theta(start_index - 1);
        path_theta(start_index) = last_theta;
    end
end

% --- Create the final matrix for Simulink ---
simulink_path_data(:, 1) = tt;
simulink_path_data(:, 2) = path_x;
simulink_path_data(:, 3) = path_y;
simulink_path_data(:, 4) = path_theta;

%% ===== 3. Post-Process to Fix Theta at Waypoints =====
% The 'atan2(0,0)' at each stop (waypoint) incorrectly returns 0.
% We overwrite these points to hold the last valid orientation.
for k = 2:n_points_total
    if (vel_x(k) == 0 && vel_y(k) == 0)
        simulink_path_data(k, 4) = simulink_path_data(k-1, 4);
    end
end
% Fix the very first point (t=0)
simulink_path_data(1, 4) = simulink_path_data(2, 4);
% Update 'path_theta' for plotting
path_theta = simulink_path_data(:, 4);

%% ===== 4. Quick Trajectory Plots =====
figure('Name', 'Desired Trajectories', 'NumberTitle', 'off');
% Plot the 2D path
subplot(2,2,1);
plot(path_x, path_y, 'b-');
hold on;
plot(waypoints(:,1), waypoints(:,2), 'ro', 'MarkerFaceColor', 'r');
grid on; axis equal;
xlabel('X Position [m]'); ylabel('Y Position [m]');
title('Desired 2D Path (Relative)');
legend('Path', 'Waypoints', Location='northwest');
% Plot X
subplot(2,2,2);
plot(tt, path_x, 'r-', tt, vel_x, 'r--');
grid on;
xlabel('Time [s]'); ylabel('X [m] or X-Vel [m/s]');
title('X-Coordinate Trajectory');
legend('x(t)', 'vx(t)', Location='northwest');
% Plot Y
subplot(2,2,3);
plot(tt, path_y, 'g-', tt, vel_y, 'g--');
grid on;
xlabel('Time [s]'); ylabel('Y [m] or Y-Vel [m/s]');
title('Y-Coordinate Trajectory');
legend('y(t)', 'vy(t)', Location='northwest');
% Plot Theta
subplot(2,2,4);
plot(tt, rad2deg(path_theta), 'm-');
grid on;
xlabel('Time [s]'); ylabel('Theta [degrees]');
title('Desired Orientation \theta_d(t)');

%% ===== 5. Run Simulation =====
disp('Starting Simulink simulation...');

% --- FIX: Assign variables to the Base Workspace ---
% Simulink model parameters (like 'Ts') and 'From Workspace' blocks
% read from the Base Workspace, not the function's workspace.
% We must explicitly send all required variables there.
assignin('base', 'simulink_path_data', simulink_path_data);
assignin('base', 'Ts', Ts);
assignin('base', 'path_x', path_x);         % <-- RIGA AGGIUNTA
assignin('base', 'path_y', path_y);         % <-- RIGA AGGIUNTA
assignin('base', 'path_theta', path_theta); % <-- RIGA AGGIUNTA
% --- End of Fix ---

out = sim("diff_drive.slx");
disp('Simulation finished.');

% --- Extract data from simulation output ---
x_data =        out.x_actual;
y_data =        out.y_actual;
theta_data =    out.theta_actual;
x_actual =      x_data.Data;
x_time =        x_data.Time;
y_actual =      y_data.Data;
y_time =        y_data.Time;
theta_aux =     theta_data.Data;
theta_actual =  squeeze(theta_aux);
theta_time =    theta_data.Time;

%% ===== 6. Final Comparison Plots =====
figure('Name', 'Desired vs. Actual Simulation Results', 'NumberTitle', 'off');
% 2D path comparison
subplot(2,2,1);
plot(path_x, path_y, 'k-', x_actual, y_actual, 'b');
grid on; axis equal;
xlabel('X Position [m]'); ylabel('Y Position [m]');
title('2D Path comparison (Relative)');
legend('Desired path', 'Actual path', Location='northwest');
% X position comparison
subplot(2,2,2);
plot(tt, path_x, 'k-', x_time, x_actual, 'r');
grid on;
xlabel('Time [s]'); ylabel('X [m]');
title('X-Coordinate Trajectory');
legend('x desired', 'x actual', Location='northwest');
% Y position comparison
subplot(2,2,3);
plot(tt, path_y, 'k-', y_time, y_actual, 'g');
grid on;
xlabel('Time [s]'); ylabel('Y [m]');
title('Y-Coordinate Trajectory');
legend('y desired', 'y actual', Location='northwest');
% Theta orientation comparison
subplot(2,2,4);
plot(tt, rad2deg(path_theta), 'k-', theta_time, rad2deg(theta_actual), 'm');
grid on;
xlabel('Time [s]'); ylabel('Theta [degrees]');
title('Orientation \theta_d(t)');
legend('theta desired','theta actual', Location='northwest');

%% ===== 7. Plot on Library Map =====
% Use the original (uninflated) map for the final plot
map = libraryMap;

% % Shift all relative paths back to their true world coordinates
% true_path_x = path_x + path(1,1);
% true_path_y = path_y + path(1,2);
% true_x_actual = x_actual + path(1,1);
% true_y_actual = y_actual + path(1,2);

figure('Name', 'Path on Map', 'NumberTitle', 'off');
show(map);
hold on;
plot(path_x, path_y, 'k-', 'LineWidth', 2);
plot(x_actual, y_actual, 'b--', 'LineWidth', 1.5);
title('Path comparison on library map');
xlabel('X [meters]');
ylabel('Y [meters]');
legend('Desired path', 'Actual path', Location='best');
hold off;

end