import numpy as np

from simple_pendulum.controllers.lqr.roa.sampling import najafi_based_sampling
from simple_pendulum.controllers.lqr.roa.utils import quad_form


class najafi_oracle:
    def __init__(self,plant,controller,n=100000):
        self.S = controller.S
        self.x_star = np.array([np.pi,0])
        self.rho, self.M = najafi_based_sampling(plant,controller,n=n)
        
    def query(self,q):
        return quad_form(self.S,q-self.x_star) < self.rho