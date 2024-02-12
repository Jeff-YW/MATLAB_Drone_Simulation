%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Visualization code for quadcopter
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Clear workspace and figures
clear all;
close all;

%% Define Flight Arena Parameters
spaceDim = 10; % Total width, length, and height of flight arena (metres)
spaceLimits = [0 spaceDim 0 spaceDim 0 0.75*spaceDim]; % Arena limits

%% Ground Image Setup
draw_ground = false; % Flag to determine if ground image is drawn
if(draw_ground)
    ground_img = imread('ground.png'); % Load ground image if enabled
end

%% Figure and Axes Setup for Drone Simulation
f1 = figure;
ax1 = gca;
view(ax1, 3); % 3D view
axis equal;
axis(spaceLimits); % Set axis limits
grid on;
grid minor;
clim(ax1, [0 spaceDim]); % Set color limits for the current axes
hold(ax1, 'on');
axis vis3d; % Freeze aspect ratio

%% Initialize Drones
num_drones = 1; % Number of drones
drones = []; % Array to store drone objects
drone_trajectory = []; % Store drone trajectories
drone_orientation = []; % Store drone orientations

% Instantiate drone objects with specified axis and arena limits
% Question 3a Quadcopter
% for i = 1:num_drones
%     fprintf('Initializing drone %d\n', i);
%     drones = [drones Drone_q3a(ax1, spaceDim, num_drones)]; % Use your drone class here
% end

% Question 3b Quadcopter
for i = 1:num_drones
    fprintf('Initializing drone %d\n', i);
    drones = [drones Drone_q3b(ax1, spaceDim, num_drones)]; % Use your drone class here
end

%% Simulation Loop
while(drones(1).time < 50)
    cla(ax1); % Clear axis for redraw
    
    % Update and draw drones
    for i = 1:num_drones
        update(drones(i)); % Update drone position and orientation
    end
    
    % Optionally draw the ground image
    if(draw_ground)
        imagesc([-spaceDim, spaceDim], [-spaceDim, spaceDim], ground_img);
    end

    % Store and plot drone trajectory
    drone_trajectory = [drone_trajectory drones.pos];
    drone_orientation = [drone_orientation drones.angles];
    
    x = drone_trajectory(1, :);
    y = drone_trajectory(2, :);
    z = drone_trajectory(3, :);
    plot3(ax1, x, y, z, 'LineWidth', 2); % Draw 3D trajectory
    plot3(ax1, x, y, zeros(1, length(z)), 'y', 'LineWidth', 1.5); % Draw projection on the ground
    
    % Apply fancy lighting (optional)
    camlight;
    
    % Update figure
    drawnow;
    
    pause(0.01); % Short pause to control simulation speed
end
