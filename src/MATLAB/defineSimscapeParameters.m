% =========================================================================
% DEFINE_SIMSCAPE_PARAMETERS
% -------------------------------------------------------------------------
% This script defines all physical parameters for the differential
% drive robot's Simscape model.
%
% These variables are loaded into the MATLAB workspace.
% =========================================================================

%% Robot Physical Properties
% Wheel properties
r_wheel = 0.02;     % Wheel radius (m)
w_wheel = 0.02;     % Wheel width (m)
% Body properties
r_body = 0.1;       % Body radius (m)
h_body = 0.05;      % Body height (m)
% Wheel mounting distances (from center of mass)
d_y = 0.07;         % y-axis distance
d_x = 0.04;         % x-axis distance
d_z = 0.02;         % z-axis distance
% Material properties
rho = 1175;         % Plastic density (kg/m^3)
% Damping
wheel_dmp = 0.003/360; % Wheel damping coefficient

%% Contact Force Parameters (Wheels vs. Ground)
% Friction coefficients (rubber)
mu_s = 0.8;         % Static friction
mu_d = 0.7;         % Dynamic friction
critical_v = 0.01;  % Critical velocity for transition
% Contact forces
contact_stiffness = 10000; % Stiffness (N/m)
contact_dmp = 30;          % Damping (N/(m/s))
tr_region = 1e-5;          % Transition region width (m/s)