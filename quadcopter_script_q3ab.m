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
spaceDim = 10;
% spaceLimits = [-spaceDim/2 spaceDim/2 -spaceDim/2 spaceDim/2 0 spaceDim/2];
spaceLimits = [0 spaceDim 0 spaceDim 0 0.75*spaceDim];

%do you want to draw a ground image on the figure?
draw_ground = true;
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

% for i = 1:num_drones
%     fprintf('i is  \n')
%     disp(i);
%     drones = [drones Drone_q3a(ax1, spaceDim, num_drones)];
% end

for i = 1:num_drones
    fprintf('i is  \n')
    disp(i);
    drones = [drones Drone_q3b(ax1, spaceDim, num_drones)];
end

% while(drones(1).time < 30.0)
while(drones(1).time < 50)

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
%     %% stopping the simulation mechanism 
%     if drones.pos(3) <= 0.05
%         if drones(1).time > 5
%             break;
%         end
%     end

    drone_trajectory = [drone_trajectory drones.pos];

    drone_oreintation = [drone_oreintation drones.angles];
        
    x = drone_trajectory(1, 1:width(drone_trajectory));
    y = drone_trajectory(2, 1:width(drone_trajectory));
    z = drone_trajectory(3, 1:width(drone_trajectory));
    plot3(ax1, x, y, z);
    plot3(ax1, x, y, zeros(1, length(z)), 'y');
    
    %apply fancy lighting (optional)
    camlight
    
    %update figure
    drawnow
   
    pause(0.01)
end

