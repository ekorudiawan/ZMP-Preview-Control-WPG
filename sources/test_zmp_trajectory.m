% This is test function for generating ZMP trajectory from given footstep input, dt, and t_step
clc;
clear;
disp("Test ZMP Trajectory");

dt = 0.01; % delta time (s)
t_step = 0.7; % timing for one step (s)

footstep = [0.00, 0.00, 0.00;
            0.25, 0.10, 0.00;
            0.50, -0.10, 0.00;
            0.75, 0.10, 0.00;
            1.00, -0.10, 0.00;
            1.25, 0.10, 0.00];

[zmp_x, zmp_y] = create_zmp_trajectory(footstep, dt, t_step);
figure('name', 'ZMP Trajectory');
plot(zmp_x, zmp_y);
        