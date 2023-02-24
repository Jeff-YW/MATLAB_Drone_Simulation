clear;
close all;

disc_sys = load("disc_sys.mat");
A_m = disc_sys.disc_sys.A;
B_m = disc_sys.disc_sys.B;



e1 = 0.9.*[1,1,1];
e2 = 0.1.*[1,1,1];
e3 = 0.95.*[1,1,1];
e4 = 0.94.*[1,1,1];
eigenvalues = [e2,e3,e1,e4];

% the pole placed state space controller matrix
K = place(A_m,B_m,eigenvalues);


%% the saved matrix attached to the answers, should be the same with the code generated K matrix
% disc_sys = load("ss_controller_matrix.mat");
% 
% K_mat = disc_sys.K;



  




