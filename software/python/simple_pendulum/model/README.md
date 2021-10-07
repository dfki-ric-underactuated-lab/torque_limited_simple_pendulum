#  Pendulum Dynamics #

The PendulumPlant class contains the kinematics and dynamics functions of the simple, torque limited pendulum.

## Theory #

<div align="center">
<img width="100" src="../../../../docs/pendulum.png">
</div>

The equations of motion of a pendulum are

```math
\begin{equation}
I\ddot{\theta} + b\dot{\theta} + c_f \text{sign}(\dot{\theta}) + mgl \sin(\theta) = \tau
\end{equation}
```

where

- $`\theta`$, $`\dot{\theta}`$, $`\ddot{\theta}`$ are the angular displacement, angular velocity and angular acceleration of the pendulum. $`\theta=0`$ means the pendulum is at its stable fixpoint (i.e. hanging down).
- $`I`$ is the inertia of the pendulum. For a point mass: $`I=ml^2`$
- $`m`$ mass of the pendulum
- $`l`$ length of the pendulum
- $`b`$ damping friction coefficient
- $`c_f`$ coulomb friction coefficient
- $`g`$ gravity (positive direction points down)
- $`\tau`$ torque applied by the motor


## API #

The pendulum plant can be initialized as follows

    pendulum = PendulumPlant(mass=1.0,
                             length=0.5,
                             damping=0.1,
                             gravity=9.81,
                             coulomb_fric=0.02,
                             inertia=None,
                             torque_limit=2.0)

where the input parameters correspond to the parameters in the equaiton of motion (1). The input inertia=None is the default and the intertia is set to the inertia of a point mass at the end of the pendulum stick ($`I=ml^2`$). Additionally, a torque_limit can be passed to the class. Torques greater than the torque_limit or smaller than -torque_limit will be cut off.

The plant can now be used to calculate the forward kinematics with

    [[x,y]] = pendulum.forward_kinematics(pos)

where pos is the angle ($`\theta`$) of interest. This function returns the (x,y) coordinates of the tip of the pendulum inside a list. The return is a list of all link coordinates of the system (as the pendulum has only one, this returns [[x,y]]).

Similarily, inverse kinematics can be computed with

    pos = pendulum.inverse_kinematics(ee_pos)

where ee_pos is a list of the end_effector coordinates [x,y]. pendulum.inverse_kinematics returns the angle of the system as a float.

Forward dynamics can be calculated with

    accn = pendulum.forward_dynamics(state, tau)

where state is the state of the pendulum $`[\theta, \dot{theta=]`$ and tau the motor torque as a float. The function returns the angular acceleration.

For inverse kinematics:

    tau = pendulum.inverse_kinematics(state, accn)

where again state is the state of the pendulum $`[\theta, \dot{\theta}]`$ and accn the acceleration. The function return the motor torque $`\tau`$ that would be neccessary to produce the desired acceleration at the specified state.

Finally, the function

    res = pendulum.rhs(t, state, tau)

returns the integrand of the equaitons of motion, i.e. the object that can be calculated with a time step to obtain the forward evolution of the system. The API of the function is written to match the API requested inside the simulator class.
t is the time which is not used in the pendulum dynamics (the dynamics do not change with time). state again is the pendulum state and tau the motor torque. res is a numpy array with shape np.shape(res)=(2,) and res = $`[\dot{\theta}, \ddot{\theta}]`$.


## Usage #

The class is inteded to be used inside the simulator class.

## Parameter Identification #

An inverse model of the robot motion dynamics is the mapping from the motion of the robot, given by the joint positions $`q(t)`$ 
$`[\dot{\theta}, \ddot{\theta}]`$ $`I {\in} `$ $`{\in}$  defr $`{\mathbb{R}}^n`$, joint velocities $`{\dot} q(t)`$ and joint accelerations $\ddot q(t)$ to the actuation torques $\tau (t)$ $\in$ $\mathbb{R}^n$ dependent on time $t$. Applying the Recursive-Newton-Euler algorithm or the Lagrange-Formalism to the a-priori known geometry of the robot \cite{siciliano2009}[^fn1], yields a theoretical model, still including unknown dynamic parameters such as the mass $m_i$, the three ﬁrst moments of inertia $m_i c_{[x|y|z],i}$ or the six second moments of inertia $I_{[xx|xy|xz|yy|yz|zz],i}$, for each body $i$ of the robot. Two additional parameters are added to the model, $F_{c,i}$ and $F_{v,i}$, in order to take joint friction into account, as coefﬁcients of a Coulomb and viscous friction model. The resulting rigid-body model thus has the form \cite{bargsten2016} [^fn2]

```math
\begin{equation}
    \tau(t)= \mathbf{Y} q(t), \dot q(t), \ddot q(t)) \; \theta,
\end{equation}
```

where $\theta$ $\in$ $\mathbb{R}^{12n}$ denotes the parameter vector with $n$ sets of parameters $\theta_i$,

```math
\begin{equation}
    \theta_i=(m_i \; m_i c_{x,i} \; m_i c_{y,i} \; m_i c_{z,i} \; I_{xx,i} \; I_{xy,i} \; I_{xz,i} \; I_{yy,i} \; I_{yz,i} \; I_{zz,i} \; F_{c,i} \; F_{v,i})^T
\end{equation}
```

 For a reference trajectory sampled at $t = kT_s, \; k$ $\in$ $1...K$ with sampling time $T_s$, an \textit{identiﬁcation matrix} $\mathit{\Phi}$ can be created

```math
\begin{equation}
    \mathit{\Phi} = \left( \begin{array}{c}
                    \mathbf{Y} (q(T_s), \; \dot q(T_s), \; \ddot q(T_s)) \\
                    ... \\
                    \mathbf{Y} (q(kT_s), \; \dot q(kT_s), \; \ddot q(kT_s)) \\
                    ... \\
                    \mathbf{Y} (q(KT_s), \; \dot q(KT_s), \; \ddot q(KT_s)) \\
                    \end{array} \right) 
\end{equation}
```

The required torques $\tau_m(kT_s)$ for model-based control can be measured using stiff position control and closely tracking the reference trajectory. A sufﬁciently rich, periodic, band-limited excitation trajectory can be obtained by modifying the parameters of a Fourier-Series as described by \cite{swevers2007}[^fn3]. The dynamic parameters $\hat{\theta}$ are estimated through least squares optimization between measured torque $\tau_m$ and computed torque $\mathit{\Phi} \hat{\theta}$;

```math
\begin{equation}
     \underset{\hat \theta}{\text{minimize}} (\mathit{\Phi} \hat{\theta} - \tau_m)^T (\mathit{\Phi} \hat{\theta} - \tau_m) .
\end{equation}
```

## References #

[^fn1]:  **Bruno Siciliano et al.** _Robotics_. Red. by Michael J. Grimble and Michael A.Johnson. Advanced Textbooks in Control and Signal Processing. London: Springer  London,  2009. ISBN:  978-1-84628-641-4  978-1-84628-642-1. DOI: 10.1007/978-1-84628-642-1. URL: http://link.springer.com/10.1007/978-1-84628-642-1 (visited on 09/27/2021).
[^fn2]: **Vinzenz Bargsten, José de Gea Fernández, and Yohannes Kassahun.** _Experimental Robot Inverse Dynamics Identification Using Classical and Machine Learning Techniques_. In: ed. by International Symposium on Robotics. OCLC: 953281127. 2016. URL: https://www.dfki.de/fileadmin/user_upload/import/8264_ISR16_Dynamics_Identification.pdf (visited on 09/27/2021).
[^fn3]: **Jan  Swevers,  Walter  Verdonck,  and  Joris  De  Schutter.** _Dynamic  ModelIdentification for Industrial Robots_. In: IEEE Control Systems27.5 (Oct.2007), pp. 58–71. ISSN: 1066-033X, 1941-000X.doi:10.1109/MCS.2007.904659. URL: https://ieeexplore.ieee.org/document/4303475/(vis-ited on 09/27/2021).
