# SwingUp the Simple Penduulum using direct Optimal Control based on the DDP algorithm:

In this package, the single pendulum swing-up is performed using the direct optimal control based on the FDDP algorithm (N.Mansard, 2019).
The script uses FDDP and BOXFddp which allows us to enforce the system's torque and velocity limits.
For online use on hardware, it is highly recommended to use BOXFddp to protect the system from possible high torque control.

The urdf model is modified to fit a pinocchio model.

# The cost model used:

For the Running Model:
State regulation + Control regulation

For the Terminal Model:
State regulation

# Dependencies:
Crocoddyl [https://github.com/loco-3d/crocoddyl]
Pinocchio [https://github.com/stack-of-tasks/pinocchio]
For the display, Gepetto [https://github.com/Gepetto/gepetto-viewer]
