function [path, libraryMap, mapInflated] = planLibraryPath()
% =========================================================================
% PLAN_LIBRARY_PATH
% -------------------------------------------------------------------------
% This function creates the library environment, inflates it,
% and runs an interactive PRM path planner.
%
% Outputs:
%   path         - [Nx2] matrix of waypoints, or empty if no path found.
%   libraryMap   - The original binaryOccupancyMap object.
%   mapInflated  - The inflated binaryOccupancyMap object.
% =========================================================================

% --- 1. CREATE LIBRARY MAP ---
% Large Map (80 rows x 120 columns)
mapMatrix = zeros(80, 120);
% Outer walls
mapMatrix(1, :) = 1;   mapMatrix(end, :) = 1;
mapMatrix(:, 1) = 1;   mapMatrix(:, end) = 1;
% Shelf properties
stackWidth = 1;      % Shelf width (m)
stackLength = 60;    % Shelf length (m)
stackTopY = 10;      % Y-position where shelves start
aisleWidth = 10;     % Space for aisles
% 4 Shelves on the LEFT of the entrance
stackColumn = aisleWidth + 1; % Start column (X=11)
for i = 1:4
    mapMatrix(stackTopY:stackTopY+stackLength, stackColumn) = 1;
    stackColumn = stackColumn + stackWidth + aisleWidth;
end
% 4 Shelves on the RIGHT of the entrance
stackColumn = 65 + aisleWidth + 1; % Start column (X=76)
for i = 1:4
    mapMatrix(stackTopY:stackTopY+stackLength, stackColumn) = 1;
    stackColumn = stackColumn + stackWidth + aisleWidth;
end
% Entrance (Centered)
mapMatrix(end, 55:65) = 0;
% Create the map object
libraryMap = binaryOccupancyMap(logical(mapMatrix), 1);

% --- 2. ROBOT PREPARATION & MAP INFLATION ---
robotRadius = 1.0; % Robot radius for path planning (m)
mapInflated = copy(libraryMap);
inflate(mapInflated, robotRadius);

% --- 3. PATH PLANNER (PRM) CONFIGURATION ---
prm = mobileRobotPRM;
prm.Map = mapInflated;
prm.NumNodes = 2500;
prm.ConnectionDistance = 10;

% --- 4. INTERACTIVE POINT SELECTION & PATH CALCULATION ---
figure('Name', 'Path Planning', 'NumberTitle', 'off');
show(libraryMap); % Show map without the PRM graph
title('Symmetric Map - Click START Point');
xlabel('X [meters]');
ylabel('Y [meters]');

disp('Click on the map to select the START point...');
startLocation = ginput(1);
hold on;
plot(startLocation(1), startLocation(2), 'ro', 'MarkerSize', 10, 'LineWidth', 2);

title('Great! Now click the END point');
disp('Click on the map to select the END point...');
endLocation = ginput(1);
plot(endLocation(1), endLocation(2), 'gx', 'MarkerSize', 10, 'LineWidth', 2);

title('Calculating path...');
drawnow; % Force MATLAB to update the plot

% Initialize path as empty
path = [];

try
    % Find the path
    path = findpath(prm, startLocation, endLocation);
    
    if isempty(path)
        disp('No path found!');
        title('No path found!');
    else
        disp('Path Found!');
        % Plot the final path
        plot(path(:,1), path(:,2), 'b-', 'LineWidth', 2);
        title('Path Found!');
        
        % --- 5. PRINT WAYPOINTS ---
        disp('--------------------------------------');
        disp('Path Waypoints (X, Y Coordinates):');
        disp(path);
        disp('--------------------------------------');
    end
    
catch ex
    disp(['Error: ' ex.message]);
    disp('You likely clicked on an obstacle. Please try again.');
    title('Error: Clicked on an obstacle!');
end
hold off;

end