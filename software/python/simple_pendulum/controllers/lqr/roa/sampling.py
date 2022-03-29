import numpy as np

from simple_pendulum.model.pendulum_plant import PendulumPlant
from simple_pendulum.controllers.lqr.lqr_controller import LQRController

from simple_pendulum.controllers.lqr.roa.plot import plot_ellipse # just for the example code
from simple_pendulum.controllers.lqr.roa.utils import sample_from_ellipsoid,quad_form

def najafi_based_sampling(plant,controller,n=10000,rho0=100,M=None,x_star=np.array([np.pi,0])):
    """Estimate the RoA for the closed loop dynamics using the method introduced in Najafi, E., Babuška, R. & Lopes, G.A.D. A fast sampling method for estimating the domain of attraction. Nonlinear Dyn 86, 823–834 (2016). https://doi.org/10.1007/s11071-016-2926-7 

    Parameters
    ----------
    plant : simple_pendulum.model.pendulum_plant
        configured pendulum plant object
    controller : simple_pendulum.controllers.lqr.lqr_controller
        configured lqr controller object
    n : int, optional
        number of samples, by default 100000
    rho0 : int, optional
        initial estimate of rho, by default 10
    M : np.array, optional
        M, such that x_barT M x_bar is the Lyapunov fct. by default None, and controller.S is used
    x_star : np.array, optional
        nominal position (fixed point of the nonlinear dynamics)

    Returns
    -------
    rho : float
        estimated value of rho
    M : np.array
        M
    """
    
    rho = rho0

    controller.set_clip()

    if M is None:
        M = np.array(controller.S)
    else:
        pass

    for i in range(n):
        # sample initial state from sublevel set
        # check if it fullfills Lyapunov conditions
        x_bar   = sample_from_ellipsoid(M,rho)
        x       = x_star+x_bar

        tau     = controller.get_control_output(x[0],x[1])[2]

        xdot    = plant.rhs(0,x,tau)

        V       = quad_form(M,x_bar)

        Vdot    = 2*np.dot(x_bar,np.dot(M,xdot))
            
        if V > rho:
            print("something is fishy")
        # V < rho is true trivially, because we sample from the ellipsoid
        if Vdot > 0.0: # if one of the lyapunov conditions is not satisfied
            rho = V

    return rho,M


if __name__ == "__main__":
    # how to use this method:

    mass = 0.57288
    length = 0.5
    damping = 0.15
    gravity = 9.81
    coulomb_fric = 0.0
    torque_limit = 2.0
    inertia = mass*length*length

    pendulum = PendulumPlant(mass=mass,
                            length=length,
                            damping=damping,
                            gravity=gravity,
                            coulomb_fric=coulomb_fric,
                            inertia=inertia,
                            torque_limit=torque_limit)


    controller = LQRController(mass=mass,
                            length=length,
                            damping=damping,
                            gravity=gravity,
                            torque_limit=torque_limit)

    rho, M = najafi_based_sampling(pendulum,controller,n=1000)

    plot_ellipse(np.pi,0,rho,M)

    print(rho)