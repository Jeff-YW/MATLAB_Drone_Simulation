%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Visualization code for quadcopter
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Cleanup Workspace and Figures
clear all;
close all;

%% Define Flight Arena Parameters
spaceDim = 20; % Define the dimensions of the flight arena
spaceLimits = [-spaceDim/2 spaceDim/2 -spaceDim/2 spaceDim/2 0 spaceDim/2]; % Define space limits

%% Ground Image Setup
draw_ground = false; % Flag for drawing ground image
if(draw_ground)
    ground_img = imread('ground.png'); % Load ground image if flag is true
end

%% Figure Setup for Drone Simulation
f1 = figure;
ax1 = gca;
view(ax1, 3); % Set 3D view
axis equal;
axis(spaceLimits); % Apply space limits to axis
grid on; % Enable grid
grid minor; % Enable minor grid lines
clim(ax1, [0 spaceDim]); % Set color limits for the axis
hold(ax1, 'on'); % Hold on to the current axis
axis vis3d; % Lock aspect ratio

%% Drone Initialization
num_drones = 1; % Number of drones
drones = []; % Initialize drones array
drone_trajectory = []; % Initialize array for storing drone trajectories
drone_orientation = []; % Corrected spelling from 'oreintation' to 'orientation'

%% Instantiate Drone Objects
% Uncomment the appropriate section for the specific quadcopter code you need

% Question 1 Quadcopter
% for i = 1:num_drones
%     fprintf('Initializing drone %d\n', i);
%     drones = [drones Drone_q1(ax1, spaceDim, num_drones)];
% end

% Question 2 Quadcopter
for i = 1:num_drones
    fprintf('Initializing drone %d\n', i);
    drones = [drones Drone_q2(ax1, spaceDim, num_drones)];
end

%% Simulation Loop
while(drones(1).time < 8)
    cla(ax1); % Clear the axis for redrawing
    
    % Update and Draw Drones
    for i = 1:num_drones
        update(drones(i)); % Update the position and orientation of each drone
    end
    
    % Optionally Draw the Ground Image
    if(draw_ground)
        imagesc([-spaceDim, spaceDim], [-spaceDim, spaceDim], ground_img); % Display ground image
    end
    
    % Store and Plot Drone Trajectory
    drone_trajectory = [drone_trajectory drones.pos]; % Update drone trajectory
    drone_orientation = [drone_orientation drones.angles]; % Update drone orientation
    
    x = drone_trajectory(1, :);
    y = drone_trajectory(2, :);
    z = drone_trajectory(3, :);
    plot3(ax1, x, y, z, 'LineWidth', 2); % Plot 3D trajectory
    plot3(ax1, x, y, zeros(1, length(z)), 'g', 'LineWidth', 1.5); % Plot ground projection in green
    
    camlight; % Apply fancy lighting (optional)
    
    drawnow; % Update figure
    pause(0.01); % Short pause for simulation speed control
end

