# ZMP preview control simulation with Python
# By : Eko Rudiawan Jamzuri
# Email : eko.rudiawan@gmail.com
# This is an implementation of ZMP preview control with feedforward method
# This Python program will calculating CoM trajectory based on ZMP trajectory input
# The Gain parameter Gi, Gx, and Gd is imported from mat file from previous calculation in Matlab

import numpy as np 
import matplotlib.pyplot as plt  
import scipy.io 

def generate_zmp_trajectory(footstep, dt, t_step):
    n_step = len(footstep)
    zmp_x = []
    zmp_y = []
    k = 0
    for i in range(0, n_step*int(t_step/dt)):
        zmp_x.append(footstep[k][0])
        zmp_y.append(footstep[k][1])
        if i != 0 and i%int(t_step/dt) == 0:
            k += 1
    return zmp_x, zmp_y

def calc_preview_control(zmp_x, zmp_y, dt, t_preview, t_calc, A_d, B_d, C_d, Gi, Gx, Gd):
    x_x = np.array([[0],
                    [0],
                    [0]])
    x_y = np.array([[0],
                    [0],
                    [0]])
    com_x = []
    com_y = []

    for i in range(0, int(t_calc/dt)):
        y_x = np.asscalar(C_d.dot(x_x))
        y_y = np.asscalar(C_d.dot(x_y))
        e_x = zmp_x[i] - y_x
        e_y = zmp_y[i] - y_y

        preview_x = 0
        preview_y = 0
        n = 0
        for j in range(i, i+int(t_preview/dt)):
            preview_x += Gd[0, n] * zmp_x[j]
            preview_y += Gd[0, n] * zmp_y[j]
            n += 1

        u_x = np.asscalar(-Gi * e_x - Gx.dot(x_x) - preview_x)
        u_y = np.asscalar(-Gi * e_y - Gx.dot(x_y) - preview_y)
        
        x_x = A_d.dot(x_x) + B_d * u_x 
        x_y = A_d.dot(x_y) + B_d * u_y

        com_x.append(x_x[0,0])
        com_y.append(x_y[0,0])

    return com_x, com_y

def main():
    print("ZMP Preview Control Simulation")
    wpg_param = scipy.io.loadmat('wpg_parameter.mat')
    A_d = wpg_param['A_d']
    zc = np.asscalar(wpg_param['zc'])
    dt = np.asscalar(wpg_param['dt'])
    t_preview = np.asscalar(wpg_param['t_preview'])
    A_d = wpg_param['A_d']
    B_d = wpg_param['B_d']
    C_d = wpg_param['C_d']
    Gi = np.asscalar(wpg_param['Gi'])
    Gx = wpg_param['Gx']
    Gd = wpg_param['Gd']

    # footstep = [[0.00, 0.00, 0.00],
    #             [0.25, 0.10, 0.00],
    #             [0.50, -0.10, 0.00],
    #             [0.75, 0.10, 0.00],
    #             [1.00, -0.10, 0.00]]

    footstep = [[0.0, 0.0, 0.0],
                [0.2, 0.06, 0.0], 
                [0.4, -0.06, 0.0],
                [0.6, 0.09, 0.0],
                [0.8, -0.03, 0.0],
                [1.3, 0.09, 0.0], 
                [1.7, -0.03, 0.0],
                [1.9, 0.09, 0.0],
                [2.0, -0.03, 0.0]]

    t_step = 0.6
    zmp_x, zmp_y = generate_zmp_trajectory(footstep, dt, t_step)
    t_calc = 4
    com_x, com_y = calc_preview_control(zmp_x, zmp_y, dt, t_preview, t_calc, A_d, B_d, C_d, Gi, Gx, Gd)
    plt.title("ZMP VS CoM Trajectory")
    plt.plot(zmp_x, zmp_y, color='green')
    plt.plot(com_x, com_y, 'x', color='red')
    plt.show()

if __name__ == "__main__":
    main()