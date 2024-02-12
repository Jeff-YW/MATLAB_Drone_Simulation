%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%  Drone class q1
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
classdef Drone_q1 < handle
    properties (Constant)
        %width, length, height offset between centre and rotors
        body = [0.6 0.6 0.0];
        
        %time interval for simulation (seconds)
        time_interval = 0.02;
        
        % size of floating window that follows drone
        axis_size = 2.;
        
        %colours of each component of drone model
        colours = [[.8 .3 .1];[.2 .2 .5];[.8 .1 .3];[.9 .6 .8];[.9 .2 .4]];
        
        %Follows the drone within the figure
        %Don't use if you are simulating more than one drone!
        %Switch to false to see the overall world view
        drone_follow = true;
    end
    properties
        %axis to draw on
        axis
        
        %length of one side of the flight arena
        spaceDim
        
        %limits of flight arena
        spaceLimits
        
        %drone position
        pos
        
        %drone rotation matrix
        R
        
        
        %Simulation time
        time
        
        %parameter to start drone in random position
        pos_offset
        
        %number of drones
        num_drones  

        % linear speed
        v
        % raw pitch yaw angles  
        angles
        % inputs the square of angular velocity
        inputs

        % angular velocity
        angular_velocity

        % linear acceleration
        x_ddot

    end
    methods
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %INSTANTIATION OF CLASS
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function obj = Drone_q1(axis, spaceDim, num_drones)
            if nargin > 1
                obj.axis = axis;
                
                obj.spaceDim = spaceDim;
                
                obj.spaceLimits = [(-spaceDim/2)+10 (spaceDim/2)-10 (-spaceDim/2)+10 (spaceDim/2)-10 10 spaceDim-10];
                
                obj.pos = [0; 0; 5];  % asked to start from z  = 5m at the beginning
                
                obj.pos_offset = [5.*(rand - 0.5),5.*(rand - 0.5),2.5.*(rand)];
                
                obj.R = [1,0,0;0,1,0;0,0,1];
                
                obj.time = 0;
                
                obj.num_drones = num_drones;

                obj.v = [0;0;0];
                % the row pitch yaw are angles， consistent with the initial
                % R
                obj.angles = [0;0;0];

                obj.inputs = [0,0,0,0];

                obj.angular_velocity = [0;0;0];

                obj.x_ddot = [0;0;0];
            else
                error('Drone not initialised correctly')
            end
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %DRAWING OF DRONE TO FIGURE
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function draw(obj)
            %how big should the moving window be
            cL = obj.axis_size;
            
            %set to false if you want to see world view
            %if(obj.drone_follow)
            %    axis([obj.pos(1)-cL obj.pos(1)+cL obj.pos(2)-cL obj.pos(2)+cL obj.pos(3)-cL obj.pos(3)+cL]);
            %end
            
            %create middle sphere
            [X Y Z] = sphere(8);
            %[X Y Z] = (obj.body(1)/5.).*[X Y Z];
            X = (obj.body(1)/5.).*X + obj.pos(1);
            Y = (obj.body(1)/5.).*Y + obj.pos(2);
            Z = (obj.body(1)/5.).*Z + obj.pos(3);
            s = surf(obj.axis,X,Y,Z);
            set(s,'edgecolor','none','facecolor',obj.colours(1,:));
            
            %create side spheres
            %front, right, back, left
            hOff = obj.body(3)/2;
            Lx = obj.body(1)/2;
            Ly = obj.body(2)/2;


            rotorsPosBody = [...
                0    Ly    0    -Ly;
                Lx    0    -Lx   0;
                hOff hOff hOff hOff];
            rotorsPosInertial = zeros(3,4);
            for i = 1:4
                rotorPosBody = rotorsPosBody(:,i);
                rotorsPosInertial(:,i) = bodyToInertial(obj,rotorPosBody);
                [X Y Z] = sphere(8);
                X = (obj.body(1)/8.).*X + obj.pos(1) + rotorsPosInertial(1,i);
                Y = (obj.body(1)/8.).*Y + obj.pos(2) + rotorsPosInertial(2,i);
                Z = (obj.body(1)/8.).*Z + obj.pos(3) + rotorsPosInertial(3,i);
                s = surf(obj.axis,X,Y,Z);
                set(s,'edgecolor','none','facecolor',obj.colours(i+1,:));
            end
            obj.axis.Title.String = ['Sim Time = ',num2str(obj.time,'%f'),' seconds'];
        end
        
        function vectorInertial = bodyToInertial(obj, vectorBody)
            vectorInertial = obj.R*vectorBody;
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %SIMULATION FUNCTIONS
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        %demo (not useful) code to show varying position and rotation
        %replace with your own functions!


        function obj = change_pos_and_orientation(obj)

            %%% thurst function
            function T = thrust(inputs, k)
            % Inputs are values for ωi^2 (sqaure angular velocity)
              T = [0; 0; k * sum(inputs)];
            end

            %%% torque function    
            function tau = torques(inputs, L, b, k)
            % Inputs are values for ωi^2 (sqaure angular velocity)
               tau = [L * k * (inputs(1) - inputs(3));
                    L * k * (inputs(2) - inputs(4));
                   b * (inputs(1) - inputs(2) + inputs(3) - inputs(4))];
            end
    
            %%% angular acceleration function    
            function omegadot = angular_acceleration(inputs, omega, I, L, b, k) 
                forces = torques(inputs, L, b, k);
                omegadot = inv(I) * (forces - cross(omega, I * omega));
            end
    
            %%% linear acceleration function
            function a = acceleration(inputs, R, xdot, m, g, k, kd) 
                gravity = [0; 0; -g];

                % R is the rotation matrix given the three Euler's angles
                T = R * thrust(inputs, k);
                % calcualte the frcitional force proportional to linear
                % speed
                Fd = -kd * xdot;

                %%% accelerataion
                a = gravity + (1 / m) * T +  (1 / m) * Fd; 

            end

            %%% function calculate the rotation matrix from Euler angles
            function R = rotation_m(thetas) 
                phi = thetas(1);
                theta = thetas(2);
                psi = thetas(3);
                term_1 = cos(psi)*cos(theta);
                term_2 = (cos(psi)*sin(phi)*sin(theta)-cos(phi)*sin(psi));
                term_3 = sin(psi)*sin(phi)+cos(phi)*cos(psi)*sin(theta);
                term_4 = cos(theta)*sin(psi);
                term_5 = cos(phi)*cos(psi)+sin(phi)*sin(psi)*sin(theta);
                term_6 = cos(phi)*sin(psi)*sin(theta) -cos(psi)*sin(phi);
                term_7= -sin(theta);
                term_8 = cos(theta)*sin(phi);
                term_9 = cos(phi)*cos(theta);

                R = [term_1, term_2, term_3 ;term_4, term_5, term_6; term_7, term_8, term_9 ];
            end
           
            t = obj.time;
            dt = obj.time_interval;

            %%% pre-defined mechanical and physical properties of the
            %%% quadcopters
            k = 1;
            g = 9.8;
            L = 0.2;
            b = 0.1;
            m = 0.2;
            k_d = 0.1;
            I = [ 1, 0, 0; 0, 1, 0; 0, 0, 0.5];



            if (0 < t) && (t <= 2)  % hovering
                 % k * (sum of the wi^2) balance the gravity acceleration
                 obj.inputs = [0.4900; 0.4900; 0.4900; 0.4900];
            elseif (2 < t) && (t <= 4)  % lifting
                % increasing the wi^2 by 15% 
                obj.inputs = [0.5635;0.5635; 0.5635; 0.5635]; 
                
            elseif (4 < t) && (t <= 8)  % diving down
                % setting the input w3^2 to 0
                obj.inputs(3) = 0;
            end

            % input is used to calculate torques, then the torque is
            % used to calculate angular accleration

            omegadot = angular_acceleration(obj.inputs, obj.angular_velocity, I, L, b, k);

            obj.angular_velocity = obj.angular_velocity + omegadot * dt;

            row = obj.angles(1);

            pitch = obj.angles(2);

            yaw = obj.angles(3);

            angle_matrix = [1,0,-sin(pitch);0,cos(row),cos(pitch)*sin(row);0,-sin(row),cos(pitch)*cos(row)];

            angle_dt = angle_matrix\obj.angular_velocity;

            obj.angles = obj.angles + angle_dt * dt;

            quad_RM = rotation_m(obj.angles);

            obj.x_ddot = acceleration(obj.inputs, quad_RM, obj.v, m, g, k, k_d);
            
            obj.v = obj.v + dt * obj.x_ddot ;


            obj.pos = obj.pos + dt * obj.v   ; % use column vector to be consistent

            obj.R = quad_RM;

%             disp("the current position is")
%             disp(obj.pos)

        end
        
        
        function update(obj)
            %update simulation time
            obj.time = obj.time + obj.time_interval;
            
            %change position and orientation of drone
            obj = change_pos_and_orientation(obj);
            
            %draw drone on figure
            draw(obj);
        end
    end
end
