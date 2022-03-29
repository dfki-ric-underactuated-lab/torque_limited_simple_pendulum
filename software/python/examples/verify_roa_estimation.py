import matplotlib.pyplot as plt
import numpy as np

from simple_pendulum.model.pendulum_plant import PendulumPlant
from simple_pendulum.controllers.lqr.lqr_controller import LQRController
from simple_pendulum.simulation.simulation import Simulator

from simple_pendulum.controllers.lqr.roa.plot import get_ellipse_patch 

from simple_pendulum.controllers.lqr.roa.sampling import najafi_based_sampling
from simple_pendulum.controllers.lqr.roa.sos import SOSequalityConstrained, SOSlineSearch
from simple_pendulum.controllers.lqr.roa.utils import sample_from_ellipsoid

roa_estimation_methods  = [najafi_based_sampling, SOSequalityConstrained, SOSlineSearch]
linestyles              = ["-",":","--"]

torque_limit = 8
init_num = 1000

mass = 0.57288
length = 0.5
damping = 0.15
gravity = 9.81
coulomb_fric = 0.0
inertia = mass*length*length


fig, ax = plt.subplots()
plt.scatter([np.pi],[0],color="black",marker="x")
plt.annotate("Goal state",(np.pi,0))

rho_max = 0

for j,mthd in enumerate(roa_estimation_methods):

    pendulum = PendulumPlant(mass=mass,
                        length=length,
                        damping=damping,
                        gravity=gravity,
                        coulomb_fric=coulomb_fric,
                        inertia=inertia,
                        torque_limit=torque_limit)

    sim = Simulator(plant=pendulum)

    controller = LQRController(mass=mass,
                            length=length,
                            damping=damping,
                            gravity=gravity,
                            torque_limit=torque_limit)

    rho,S = mthd(pendulum,controller)

    if (rho > rho_max):
        rho_max = rho

    if (rho != 0):
        p = get_ellipse_patch(np.pi,0,rho,S,linec= "black",linest=linestyles[j])
        ax.add_patch(p)

for i in range(0,init_num):   
    x_i = sample_from_ellipsoid(S,rho_max)

    controller.set_goal([np.pi, 0])
    controller.set_clip()
    dt = 0.01
    t_final = 2.0

    T, X, U = sim.simulate(t0=0.0, 
                    x0=[x_i[0]+ np.pi,x_i[1]],
                    tf=t_final,
                    dt=dt,
                    controller=controller,
                    integrator="runge_kutta"
                    )
        
    if (round(np.asarray(X).T[0][-1]) == round(np.pi) and round(np.asarray(X).T[1][-1]) == 0):
        plt.scatter([x_i[0]+ np.pi],[x_i[1]],color="green",marker="o")
    else:
        plt.scatter([x_i[0]+ np.pi],[x_i[1]],color="red",marker="o")

plt.grid(True)
plt.show()

#plt.plot(np.asarray(X).T[0],np.asarray(X).T[1], color = "black")
#T, X, U = sim.simulate_and_animate(t0=0.0,
#                                   x0=[x_i[0]+ np.pi,x_i[1]],
#                                   tf=t_final,
#                                   dt=dt,
#                                   controller=controller,
#                                   integrator="runge_kutta"
#                                   )