% ZMP Preview Control Simulation
% By : Eko Rudiawan Jamzuri
% Email : eko.rudiawan@gmail.com
% To deep understanding about this code, please refer to the paper

clc;
clear;
disp("Test Preview Control");

zc = 0.22; % LIPM height
dt = 0.01; % delta time (s)
t_step = 0.7; % timing for one step (s)
t_preview = 1; % timing for preview (s)
t_calc = 2; % timing parameter for simulation (s), this value will be override automatically by calculation to prevent error

% Need to be tuned manually
% Tune these two parameters until you get proper CoM trajectory
Qe = 1;
R = 1e-2;

% Given footstep input
% x, y, and theta
% footstep = [0.00, 0.00, 0.00;
%             0.25, 0.10, 0.00;
%             0.50, -0.10, 0.00;
%             0.75, 0.10, 0.00;
%             1.00, -0.10, 0.00];
   
footstep = [0.0 0.0 0.0; 
            0.2 0.06 0.0; 
            0.4 -0.06 0.0; 
            0.6 0.09 0.0; 
            0.8 -0.03 0.0; 
            1.3 0.09 0.0; 
            1.7 -0.03 0.0; 
            1.9 0.09 0.0; 
            2.0 -0.03 0.0];

% Automatic calculation for simulation time
t_calc = length(footstep) * t_step - t_preview - dt;

% Generating ZMP trajectory
[zmp_x, zmp_y] = create_zmp_trajectory(footstep, dt, t_step);

% Getting parameter and gain
[A_d, B_d, C_d, Gi, Gx, Gd] = get_preview_control_parameter(zc, dt, t_preview, Qe, R);

% Simulating for t_calc seconds
[com_x, com_y] = calc_preview_control(zmp_x, zmp_y, dt, t_preview, t_calc, A_d, B_d, C_d, Gi, Gx, Gd);

% Plot ZMP and CoM trajectory
figure('name','ZMP X-Axis');
plot(zmp_x);
hold;
plot(com_x, 'o');
figure('name','ZMP Y-Axis');
plot(zmp_y);
hold;
plot(com_y, 'o');
figure('name','ZMP VS CoM');
plot(zmp_x, zmp_y);
hold;
plot(com_x, com_y, 'x');

% Save parameter to mat file
% We can import this parameter to Python or C++ for real usage
save('wpg_parameter', 'zc', 'dt', 't_preview', 'A_d', 'B_d', 'C_d', 'Gi', 'Gx', 'Gd');
