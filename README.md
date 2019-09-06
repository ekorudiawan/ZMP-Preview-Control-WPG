# ZMP-Preview-Control-WPG
ZMP Preview Control Walking Pattern Generation for Biped Humanoid Robot

This is a source code of biped walking pattern generator with ZMP preview control.
The source code is written in Matlab and Python. 
Matlab is used to calculate gain matrix then Python is used to simulate the system by feedforward control.

**Matlab result example:**

ZMP and CoM Trajectory in x direction

![ZMP VS CoM in x direction](/images/matlab_zmp_x.bmp)

ZMP and CoM Trajectory in y direction

![ZMP VS CoM in y direction](/images/matlab_zmp_y.bmp)

ZMP and CoM Trajectory in x and y direction

![ZMP VS CoM in y direction](/images/matlab_zmp_com.bmp)

**Python result example:**

ZMP and CoM trajectory in x and y direction

![ZMP VS CoM in x and y direction](/images/python_zmp_com.png)

For deep understanding please refer to the following papers. All variable in source code refer to notation in these papers.

1. [Kajita, Shuuji, et al. "Biped walking pattern generation by using preview control of zero-moment point." 2003 IEEE International Conference on Robotics and Automation (Cat. No. 03CH37422). Vol. 2. IEEE, 2003.](https://ieeexplore.ieee.org/iel5/8794/27834/01241826.pdf)
2. [Katayama, Tohru, et al. "Design of an optimal controller for a discrete-time system subject to previewable demand." International Journal of Control 41.3 (1985): 677-699.](http://people.csail.mit.edu/katiebyl/kb/DW2008/papers_of_tangential_interest/katayama85.pdf)
