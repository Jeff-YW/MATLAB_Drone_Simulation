%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%  Code modified by Francisco Vasconcelos from
%%%%
%%%%  Drone class, feel free to add functionality as you see fit
%%%%  Author: Daniel Butters
%%%%  Date: 16/11/17
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
classdef Drone_q3a < handle
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
        
        %drone position, x1
        pos
        
        %drone rotation matrix
        R
        
        
        %Simulation time
        time
  
        
        %number of drones
        num_drones  

        % raw pitch yaw angles, x3  
        angles

        % angular velocity, x4
        angular_velocity


        % linear speed, x2
        x_dot

        % thurst force in z axis
        delta_T

        % torque force (3 vectors)
        Tau
        % counting the current target
        count

        % hovering at 5 5 5
        h_duration



    end
    methods
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %INSTANTIATION OF CLASS
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function obj = Drone_q3a(axis, spaceDim, num_drones)
            if nargin > 1
                obj.axis = axis;
                
                obj.spaceDim = spaceDim;
                
                obj.spaceLimits = [(-spaceDim/2)+10 (spaceDim/2)-10 (-spaceDim/2)+10 (spaceDim/2)-10 10 spaceDim-10];
                
                obj.pos = [0;0;0];  % in question 3, the drone starts from (0,0,0) (x,y,z) respectively Cartesian coordinates

                obj.R = [1,0,0;0,1,0;0,0,1];
                
                obj.time = 0;
                
                obj.num_drones = num_drones;

                % the row pitch yaw are angles， consistent with the initial
                % rotation matrix
                obj.angles = [0;0;0];

                obj.angular_velocity = [0;0;0];

                obj.x_dot = [0;0;0];

                obj.delta_T = -0.2*9.8;  % initially delta_T = -mg... since we define the T initially as 0...


                % defined input, torque x,y,z components
                obj.Tau = [0;0;0];
              
                % start with the first target position
                obj.count = 1;

                obj.h_duration = 0;



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

        function target = produce_target_point(obj,Num)
                          %%% calculate target point
           arc_rad = 2.5;
%             Num = 10;
            cir_angle_1 = linspace(0, -pi/2, Num);
            intermediate_1 = [2.5 + arc_rad*cos(cir_angle_1); 5 + arc_rad*sin(cir_angle_1); repelem(5, Num) ];
            
            cir_angle_2 = linspace(-pi/2, -pi, Num);
            intermediate_2 = [2.5 + arc_rad*cos(cir_angle_2); 5 + arc_rad*sin(cir_angle_2); repelem(5, Num) ];
            
            cir_angle_3 = linspace(-pi, -3*pi/2, Num);
            intermediate_3 = [2.5 + arc_rad*cos(cir_angle_3); 5 + arc_rad*sin(cir_angle_3); repelem(5, Num) ];
            
            cir_angle_4 = linspace(-3*pi/2, -2*pi, Num);
            intermediate_4 = [2.5 + arc_rad*cos(cir_angle_4); 5 + arc_rad*sin(cir_angle_4); repelem(5, Num) ];

            
            inter_speed_1 = [ 0.2* sin(cir_angle_1);-0.2* cos(cir_angle_1); repelem(0, Num) ];

            inter_speed_2 = [ 0.2*sin(cir_angle_2); -0.2* cos(cir_angle_2);repelem(0, Num) ];

            inter_speed_3 = [ 0.2* sin(cir_angle_3); -0.2* cos(cir_angle_3);repelem(0, Num) ];

            inter_speed_4 = [ 0.2*sin(cir_angle_4); -0.2* cos(cir_angle_4);repelem(0, Num) ];

            fixed_target_last = [5;5;0];
            
            full_tar = [intermediate_1, intermediate_2, intermediate_3, intermediate_4, fixed_target_last];
            
            tar_size = size(full_tar);
            
            % select the number of target locations
            tar_len = tar_size(2); 

            zero_speed = [0;0;0]; % hover at place 5,5,5

            inter_speed_1(:,1) =  zero_speed;   % hovering at (5,5,5) state of speed is (0 , 0, 0)

            minor_linear_speed = [0.05; 0.05; 0.05];    % final target point, speed is smaller than 0.1 m/s, target state defined for final point (5,5,0)

            stack_speed = [ inter_speed_1, inter_speed_2, inter_speed_3, inter_speed_4, minor_linear_speed];

            stack_zero = zeros(6,tar_len);
      
            target = [full_tar; stack_speed ;stack_zero];

        end

        function obj = change_pos_and_orientation(obj)
            
            Num = 10;
            full_tar_stack = produce_target_point(obj,Num);
            
            tar_size = size(full_tar_stack);
            
            % select the number of target locations
            tar_len = tar_size(2); 

            function T = thrust(delta_thurst, k, m, g)
            % Inputs are values for ωi2
              T = [0; 0; k * (delta_thurst + m*g)];
            end

            function tau = torques(torque_column_array)
               tau = [torque_column_array(1)
                    torque_column_array(2)
                   torque_column_array(3)];
            end

            function omegadot = angular_acceleration(torques_calculated, omega, I) 

                omegadot = I \ (torques_calculated - cross(omega, I * omega));
            end

    
            function a = acceleration(inputs, R, xdot, m, g, k, kd)     % R the rotation matrix
                gravity = [0; 0; -g];
                delta_thurst = inputs(1);
                T = R * thrust(delta_thurst, k, m, g);
                Fd = -kd * xdot;
                a = gravity + (1 / m) * T + (1 / m) * Fd; 

            end


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

            % defined constant
            k = 1;
            g = 9.8;
            L = 0.2;
            b = 0.1;
            m = 0.2;
            k_d = 0.1;
            I = [ 1, 0, 0; 0, 1, 0; 0, 0, 0.5];



            %%% Load the calculated K_matrix from the saved space
            
            disc_sys = load("ss_controller_matrix.mat");
            

            K_mat = disc_sys.K;
            
            %%% threshold norm of position error to target position
            e_threshold = 0.05;


            % the linear state space equation inputs are stacked as (1,4)
            % column matrix
            target = full_tar_stack(:,obj.count);
            
            Curr_state = [obj.pos; obj.x_dot; obj.angles; obj.angular_velocity];

            %% Guassian Noise added 3b

%             % noise added to the position 4%
%             Curr_state(1:3) = Curr_state(1:3) + Curr_state(1:3) .* 0.04 .*randn(3,1);
% 
%             % noise added to the linear speed 1%
%             Curr_state(4:6) = Curr_state(4:6) + Curr_state(4:6) .* 0.01 .*randn(3,1);
%             % noise added to the orientation 2%  
%             Curr_state(7:9) = Curr_state(7:9) + Curr_state(7:9) .* 0.02 .*randn(3,1);
% 
%             % noise added to the angular speed 0.5%
%             Curr_state(10:12) = Curr_state(10:12) + Curr_state(10:12) .* 0.005 .* randn(3,1);

            state_e = Curr_state - target;

            % calculating the nomalized single error 
            e = norm(state_e(1:3));
            
            % Criteria updating the targeted state

            if e < e_threshold && obj.count < tar_len
                if obj.count == 1 && e < e_threshold && obj.h_duration <= 5
                    obj.h_duration = obj.h_duration + obj.time_interval;

                else
                    obj.count = obj.count + 1;
                end
            end


            % controller update the input at the current time step
            Input = - K_mat*state_e;

            %update the input in the obj function after state control

            obj.delta_T = Input(1);

            obj.Tau = Input(2:4);

            %%% use the controlled new delat T and tau to calculate the
            %%% acceleration and speed and position and angles...

            omegadot = angular_acceleration(obj.Tau, obj.angular_velocity, I);
            

            obj.angular_velocity = obj.angular_velocity + omegadot * dt;

            %%% Extract the angles of the drone 
            row = obj.angles(1);

            pitch = obj.angles(2);

            yaw = obj.angles(3);

            angle_matrix = [1,0,-sin(pitch);0,cos(row),cos(pitch)*sin(row);0,-sin(row),cos(pitch)*cos(row)];

            %%% using the state space eqaution 3: x3_dot = K^(-1) * x4
            angle_dt = angle_matrix\obj.angular_velocity;


            obj.angles = obj.angles + angle_dt * dt;


            quad_RM = rotation_m(obj.angles);
            
            
            obj.x_dot = obj.x_dot + dt * acceleration(Input, quad_RM, obj.x_dot, m, g, k, k_d) ;


            obj.pos = obj.pos + dt * obj.x_dot  ; % use column vector to be consistent


            obj.R = quad_RM;
              

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