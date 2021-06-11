# SwingUp the Simple Pendulum using direct Optimal Control based on the FDDP and BoxDDP algorithm:

In this package, the single pendulum swing-up is performed using the direct optimal control based on the FDDP algorithm (N.Mansard, 2019).

The script uses FDDP and BOXFddp which allows us to enforce the system's torque and velocity limits.
For online use on hardware, it is highly recommended to use BOXFddp to protect the system from possible high torque control.

The urdf model is modified to fit a pinocchio model.

# Cost models used:

 * The costs functions for the **Running model** is written as :

```math
\begin{equation}
  Costl = {\sum}_{n=1}^{T-1} \alpha_n \Phi_n(q,\dot{q},\Tau),
\end{equation}
```

With the following costs and weights, $`t_S`$ denoting the final time horizon.

1. _**Torque minimization**_: Minimization of the joint torques for realistic dynamic motions.

$`\Phi_{1} =  \parallel \Tau (t) \parallel ^{2}_2,   \alpha_1 = 1e-3`$

2. _**Posture regularization**_:  giving as input only the final reference posture. 

$`\Phi_{2} = \parallel q(t)-q^{ref}(t_{s-1})\parallel ^{2}_2 ,   \alpha_2 = 0.000010`$



* The costs functions for the **Terminal model** are applied to only one node (the terminal node) and is written as :


```math
 \begin{equation}
  Costl_T =  \alpha_T \Phi_T(q,\dot{q}),
 \end{equation}
```

With the following cost and weight, $`T = t_{final}`$ the final time horizon.

1. _**Posture regularization**_: giving as input only the final reference posture. 


    $`\Phi_{3} = \parallel q(T)-q^{ref}(T)\parallel^{2}_2 ,   \alpha_{3} = 1e10`$




The weights $`\alpha_i`$ for this optimization problem are determined experimentally. 


# Dependencies:

Crocoddyl [https://github.com/loco-3d/crocoddyl]

Pinocchio [https://github.com/stack-of-tasks/pinocchio]

For the display, Gepetto [https://github.com/Gepetto/gepetto-viewer]
