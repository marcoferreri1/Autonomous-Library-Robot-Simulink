% =========================================================================
% MAIN_RUN_SIMULATION
% -------------------------------------------------------------------------
% This is the top-level script to run the full differential drive
% robot simulation.
%
% It executes the following steps:
%   1. Defines all physical parameters for the Simscape model.
%   2. Creates the library map and runs an interactive path planner (PRM).
%   3. Generates a smooth trajectory from the planned path.
%   4. Runs the "diff_drive.slx" Simulink model.
%   5. Plots all comparison results.
% =========================================================================

% --- 1. INITIAL SETUP ---
clear;
clc;
close all;

% --- 2. DEFINE SIMSCAPE PARAMETERS ---
% Run the script to load all physical robot parameters (mass, friction,
% dimensions, etc.) into the MATLAB workspace.
defineSimscapeParameters;

% --- 3. PLAN PATH ---
% Call the path planning function. This will:
%   - Create the binary occupancy map.
%   - Inflate it for the robot's radius.
%   - Open a figure and ask the user to click a start and end point.
%   - Return the waypoints, the original map, and the inflated map.
[path, libraryMap, mapInflated] = planLibraryPath;

% --- 4. RUN SIMULATION & PLOT ---
% Only proceed if the path planner found a valid path.
if ~isempty(path)
    % Call the trajectory generation and simulation function.
    % This function takes the waypoints and maps, generates the
    % smooth trajectory, runs the .slx model, and plots all results.
    generateTrajectoryAndRunSim(path, libraryMap, mapInflated);
else
    disp('Simulation skipped: No valid path was found.');
end