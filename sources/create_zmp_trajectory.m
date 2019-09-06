function [zmp_x,zmp_y] = create_zmp_trajectory(footstep, dt, t_step)
n_step = length(footstep);
k = 1;
zmp_x = [];
zmp_y = [];
for i=0:dt:n_step*t_step
    zmp_x = [zmp_x footstep(k,1)];
    zmp_y = [zmp_y footstep(k,2)];
    if i ~=0 && mod(i,t_step) == 0
       k = k+1;
    end
end
end

