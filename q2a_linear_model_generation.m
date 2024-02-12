%% Question 2a
clear
close all

syms x y z x_dot y_dot z_dot phi theta psi omega_x omega_y omega_z delta_T tau_1 tau_2 tau_3

time_interval = 0.02;

k = 1;
g = 9.8;
L = 0.2;
b = 0.1;
m = 0.2;
k_d = 0.1;
I = [ 1, 0, 0; 0, 1, 0; 0, 0, 0.5];
R_0 = [1,0,-sin(theta);0,cos(phi),cos(theta)*sin(phi); 0, -sin(theta),cos(theta)*cos(phi)];


% state vector
x = [x; y; z; x_dot; y_dot; z_dot; phi; theta; psi; omega_x; omega_y; omega_z];

% input
u = [delta_T; tau_1; tau_2; tau_3];

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


x_1_dot = [x_dot; y_dot; z_dot];

x_2_dot = [0;0;-g] + (1/m)*R*[0;0;delta_T + m*g] - (1/m) *k_d* [x_dot; y_dot; z_dot];

x_3_dot = R_0 * [omega_x; omega_y; omega_z];

forces = [tau_1; tau_2; tau_3];

x_4 = [omega_x omega_y omega_z];  % angular velocity

x_4_dot = [forces(1)* 1;forces(2)* 1;forces(3)* 2] - [(1-0.5/1)*x_4(2)*x_4(3);(0.5-1/1)*x_4(1)*x_4(3);(1-1/0.5)*x_4(1)*x_4(2)];




% dynamics x_dot
xd = [x_1_dot;x_2_dot;x_3_dot;x_4_dot];

% The operating point here is theta=0; thetadot=0; d=anything
% Therefore we do not neet do do any change of variables and can compute the jacobians directly

Aj = jacobian(xd,x);
Bj = jacobian(xd,u);

A = subs(Aj,theta, 0);
A = subs(A,psi, 0);
A = subs(A,phi, 0);
A = subs(A,delta_T, 0);
A = subs(A,omega_x, 0);
A = subs(A,omega_y, 0);
A = subs(A,omega_z, 0);
A = double(A);

B = subs(Bj,theta, 0);
B = subs(B,psi, 0);
B = subs(B,phi, 0);
B = double(B);

C = eye (12);
D = zeros(12,4);

cont_sys = ss(A,B,C,D);

% the generated discrtized model, which can then be saved as'disc_sys.mat
% and used in question 2b
disc_sys = c2d(cont_sys,time_interval,'zoh');