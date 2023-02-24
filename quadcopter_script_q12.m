%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%  Code modified by Francisco Vasconcelos from
%%%%
%%%%  Visualisation code for quadcopter 
%%%%  Author: Daniel Butters
%%%%  Date: 16/11/17
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all;
close all;

%Define total width, length and height of flight arena (metres)
spaceDim = 20;
spaceLimits = [-spaceDim/2 spaceDim/2 -spaceDim/2 spaceDim/2 0 spaceDim/2];

%do you want to draw a ground image on the figure?
draw_ground = false;
if(draw_ground)
    ground_img = imread('ground.png');
end


%figure to display drone simulation
f1 = figure;
ax1 = gca;
view(ax1, 3);
axis equal;
axis(spaceLimits)
grid ON
grid MINOR
clim(ax1, [0 spaceDim]);
hold(ax1,'on')
axis vis3d



num_drones = 1;

%instantiate a drone object, input the axis and arena limits
drones = [];
drone_trajectory = [];
drone_oreintation = [];


% Uncomment the below to load question 1 quad copter code, and comment the
% above question 2 code
for i = 1:num_drones
    fprintf('i is  \n')
    disp(i);
    drones = [drones Drone_q1(ax1, spaceDim, num_drones)];
end

% Uncomment the below to load question 2 quad copter code, and comment the
% above question 1 code

% for i = 1:num_drones
%     fprintf('i is  \n')
%     disp(i);
%     drones = [drones Drone_q2(ax1, spaceDim, num_drones)];
% end



while(drones(1).time < 8)

    %clear axis
    cla(ax1);
    
    %update and draw drones
    for i = 1:num_drones
        update(drones(i));
    end
    
    %optionally draw the ground image
    if(draw_ground)
        imagesc([-spaceDim,spaceDim],[-spaceDim,spaceDim],ground_img);
    end
    % update the drone position, append it to the matrix
    drone_trajectory = [drone_trajectory drones.pos];
    % update the drone orientation, append it to the matrix
    drone_oreintation = [drone_oreintation drones.angles];
    x = drone_trajectory(1, 1:width(drone_trajectory));
    y = drone_trajectory(2, 1:width(drone_trajectory));
    z = drone_trajectory(3, 1:width(drone_trajectory));
    plot3(ax1, x, y, z);
    plot3(ax1, x, y, zeros(1, length(z)), 'g');

    %apply fancy lighting (optional)
    camlight
    
    %update figure
    drawnow
    pause(0.01)
end

