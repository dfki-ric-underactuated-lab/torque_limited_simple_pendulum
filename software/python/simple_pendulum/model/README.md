#  Pendulum Dynamics #

The PendulumPlant class contains the kinematics and dynamics functions of the simple, torque limited pendulum.

## Theory #

<div align="center">
<img width="100" src="../../../../docs/pendulum.png">
</div>

The equations of motion of a pendulum are

<img src="https://render.githubusercontent.com/render/math?math=I\ddot{\theta} %2B b\dot{\theta} %2B c_f \text{sign}(\dot{\theta}) %2B mgl \sin(\theta) = \tau">

where

- <img src="https://render.githubusercontent.com/render/math?math=\theta">, <img src="https://render.githubusercontent.com/render/math?math=\dot{\theta}">, <img src="https://render.githubusercontent.com/render/math?math=\ddot{\theta}"> are the angular displacement, angular velocity and angular acceleration of the pendulum. <img src="https://render.githubusercontent.com/render/math?math=\theta=0"> means the pendulum is at its stable fixpoint (i.e. hanging down).
- <img src="https://render.githubusercontent.com/render/math?math=I"> is the inertia of the pendulum. For a point mass: <img src="https://render.githubusercontent.com/render/math?math=I=ml^2">
- <img src="https://render.githubusercontent.com/render/math?math=m"> mass of the pendulum
- <img src="https://render.githubusercontent.com/render/math?math=l"> length of the pendulum
- <img src="https://render.githubusercontent.com/render/math?math=b"> damping friction coefficient
- <img src="https://render.githubusercontent.com/render/math?math=c_f"> coulomb friction coefficient
- <img src="https://render.githubusercontent.com/render/math?math=g"> gravity (positive direction points down)
- <img src="https://render.githubusercontent.com/render/math?math=\tau"> torque applied by the motor


## API #

The pendulum plant can be initialized as follows

    pendulum = PendulumPlant(mass=1.0,
                             length=0.5,
                             damping=0.1,
                             gravity=9.81,
                             coulomb_fric=0.02,
                             inertia=None,
                             torque_limit=2.0)

where the input parameters correspond to the parameters in the equaiton of motion (1). The input inertia=None is the default and the intertia is set to the inertia of a point mass at the end of the pendulum stick (<img src="https://render.githubusercontent.com/render/math?math=I=ml^2">). Additionally, a torque_limit can be passed to the class. Torques greater than the torque_limit or smaller than -torque_limit will be cut off.

The plant can now be used to calculate the forward kinematics with

    [[x,y]] = pendulum.forward_kinematics(pos)

where pos is the angle (<img src="https://render.githubusercontent.com/render/math?math=\theta">) of interest. This function returns the (x,y) coordinates of the tip of the pendulum inside a list. The return is a list of all link coordinates of the system (as the pendulum has only one, this returns [[x,y]]).

Similarily, inverse kinematics can be computed with

    pos = pendulum.inverse_kinematics(ee_pos)

where ee_pos is a list of the end_effector coordinates [x,y]. pendulum.inverse_kinematics returns the angle of the system as a float.

Forward dynamics can be calculated with

    accn = pendulum.forward_dynamics(state, tau)

where state is the state of the pendulum <img src="https://render.githubusercontent.com/render/math?math=[\theta, \dot{theta=]"> and tau the motor torque as a float. The function returns the angular acceleration.

For inverse kinematics:

    tau = pendulum.inverse_kinematics(state, accn)

where again state is the state of the pendulum <img src="https://render.githubusercontent.com/render/math?math=[\theta, \dot{\theta}]"> and accn the acceleration. The function return the motor torque <img src="https://render.githubusercontent.com/render/math?math=\tau"> that would be neccessary to produce the desired acceleration at the specified state.

Finally, the function

    res = pendulum.rhs(t, state, tau)

returns the integrand of the equaitons of motion, i.e. the object that can be calculated with a time step to obtain the forward evolution of the system. The API of the function is written to match the API requested inside the simulator class.
t is the time which is not used in the pendulum dynamics (the dynamics do not change with time). state again is the pendulum state and tau the motor torque. res is a numpy array with shape np.shape(res)=(2,) and res = <img src="https://render.githubusercontent.com/render/math?math=[\dot{\theta}, \ddot{\theta}]">.


## Usage #

The class is inteded to be used inside the simulator class.

## Parameter Identification #

The rigid-body model dervied from a-priori known geometry as described by [^fn1] has the form

<div align="center">
<img src="https://render.githubusercontent.com/render/math?math=%5Ctau(t)%3D%20%5Cmathbf%7BY%7D%20%5Cleft(%5Ctheta(t)%2C%20%5Cdot%20%5Ctheta(t)%2C%20%5Cddot%20%5Ctheta(t)%5Cright)%20%5C%3B%20%5Clambda%2C">
</div>

<!-- $$\tau(t)= \mathbf{Y} \left(\theta(t), \dot \theta(t), \ddot \theta(t)\right) \; \lambda,$$ -->


where actuation torques <img src="https://render.githubusercontent.com/render/math?math=\tau (t)">, joint positions <img src="https://render.githubusercontent.com/render/math?math=\theta(t)">, velocities <img src="https://render.githubusercontent.com/render/math?math=\dot \theta (t)"> and accelerations <img src="https://render.githubusercontent.com/render/math?math=\ddot \theta(t)"> depend on time <img src="https://render.githubusercontent.com/render/math?math=t"> and <img src="https://render.githubusercontent.com/render/math?math=\lambda"> <img src="https://render.githubusercontent.com/render/math?math=\in"> <img src="https://render.githubusercontent.com/render/math?math=\mathbb{R}^{6n}"> denotes the parameter vector. Two additional parameters for Coulomb and viscous friction are added to the model, <img src="https://render.githubusercontent.com/render/math?math=F_{c,i}"> and <img src="https://render.githubusercontent.com/render/math?math=F_{v,i}">, in order to take joint friction into account [^fn2]. The required torques for model-based control can be measured using stiff position control and closely tracking the reference trajectory. A sufﬁciently rich, periodic, band-limited excitation trajectory is obtained by modifying the parameters of a Fourier-Series as described by [^fn3]. The dynamic parameters <img src="https://render.githubusercontent.com/render/math?math=\hat{\lambda}"> are estimated through least squares optimization between measured torque and computed torque

<div align="center">
<img src="https://render.githubusercontent.com/render/math?math=%5Chat%7B%5Clambda%7D%20%3D%20%5Cunderset%7B%5Clambda%7D%7B%5Ctext%7Bargmin%7D%7D%20%5Cleft(%20(%5Cmathit%7B%5CPhi%7D%20%5Clambda%20-%20%5Ctau_m)%5ET%20(%5Cmathit%7B%5CPhi%7D%20%5Clambda%20-%20%5Ctau_m)%20%5Cright)">
</div>

<!-- $$\hat{\lambda} = \underset{\lambda}{\text{argmin}} \left( (\mathit{\Phi} \lambda - \tau_m)^T (\mathit{\Phi} \lambda - \tau_m) \right),$$ -->


where <img src="https://render.githubusercontent.com/render/math?math=\mathit{\Phi}"> denotes the identiﬁcation matrix.


## References #

[^fn1]:  **Bruno Siciliano et al.** _Robotics_. Red. by Michael J. Grimble and Michael A.Johnson. Advanced Textbooks in Control and Signal Processing. London: Springer  London,  2009. ISBN:  978-1-84628-641-4  978-1-84628-642-1. DOI: 10.1007/978-1-84628-642-1. URL: http://link.springer.com/10.1007/978-1-84628-642-1 (visited on 09/27/2021).
[^fn2]: **Vinzenz Bargsten, José de Gea Fernández, and Yohannes Kassahun.** _Experimental Robot Inverse Dynamics Identification Using Classical and Machine Learning Techniques_. In: ed. by International Symposium on Robotics. OCLC: 953281127. 2016. URL: https://www.dfki.de/fileadmin/user_upload/import/8264_ISR16_Dynamics_Identification.pdf (visited on 09/27/2021).
[^fn3]: **Jan  Swevers,  Walter  Verdonck,  and  Joris  De  Schutter.** _Dynamic  ModelIdentification for Industrial Robots_. In: IEEE Control Systems27.5 (Oct.2007), pp. 58–71. ISSN: 1066-033X, 1941-000X.doi:10.1109/MCS.2007.904659. URL: https://ieeexplore.ieee.org/document/4303475/(vis-ited on 09/27/2021).
