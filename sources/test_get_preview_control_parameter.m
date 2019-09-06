% This is test function for getting parameter and gain from ZMP preview control

clc;
clear;

zc = 0.22; % LIPM height
dt = 0.01; % delta time (s)
t_preview = 1; % timing for preview (s)

Qe = 1e-4;
R = 1e-6;

[A_d, B_d, C_d, Gi, Gx, Gd] = get_preview_control_parameter(zc, dt, t_preview, Qe, R);

% Printout matrix A, B, C and gain 
disp(A_d);
disp(B_d);
disp(C_d);
disp(Gi);
disp(Gx);
disp(Gd);