import matplotlib.pyplot as plt
import numpy as np

from simple_pendulum.model.pendulum_plant import PendulumPlant
from simple_pendulum.controllers.lqr.lqr_controller import LQRController

from simple_pendulum.controllers.lqr.roa.plot import get_ellipse_patch 

from simple_pendulum.controllers.lqr.roa.sampling import najafi_based_sampling
from simple_pendulum.controllers.lqr.roa.sos import SOSequalityConstrained, SOSlineSearch

roa_estimation_methods  = [najafi_based_sampling, SOSequalityConstrained, SOSlineSearch]
linestyles              = ["-",":","--"]

torque_limits = [0.1,0.5,1,2,3]
colors        = ["red","darkorange","gold","yellow","yellowgreen"]

mass = 0.57288
length = 0.5
damping = 0.15
gravity = 9.81
coulomb_fric = 0.0
inertia = mass*length*length


fig, ax = plt.subplots()
plt.scatter([np.pi],[0],color="black",marker="x")


for i,torque_limit in enumerate(torque_limits):
    for j,mthd in enumerate(roa_estimation_methods):

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

        rho,S = mthd(pendulum,controller)

        if (rho != 0):
            p = get_ellipse_patch(np.pi,0,rho,S,linec=colors[i],linest=linestyles[j])
            ax.add_patch(p)

plt.grid(True)
plt.show()
