import numpy as np


class PendulumPlant:
    def __init__(self, mass=1.0, length=0.5, damping=0.1, gravity=9.81,
                 coulomb_fric=0.0, inertia=None, torque_limit=np.inf):
        self.m = mass
        self.l = length
        self.b = damping
        self.g = gravity
        self.coulomb_fric = coulomb_fric
        if inertia is None:
            self.inertia = mass*length*length
        else:
            self.inertia = inertia

        self.torque_limit = torque_limit

        self.dof = 1
        self.n_actuators = 1
        self.base = [0, 0]
        self.n_links = 1
        self.workspace_range = [[-1.2*self.l, 1.2*self.l],
                                [-1.2*self.l, 1.2*self.l]]

    def forward_kinematics(self, pos):
        """
        forward kinematics, origin at fixed point
        """
        ee_pos_x = self.l * np.sin(pos)
        ee_pos_y = -self.l*np.cos(pos)
        return [[ee_pos_x, ee_pos_y]]

    def inverse_kinematics(self, ee_pos):
        """
        inverse kinematics, end-effector coordinates with origin at fixed point
        """
        pos = np.arctan2(ee_pos[1], ee_pos[0])
        return pos

    def forward_dynamics(self, state, tau):
        accn = (tau - self.m * self.g * self.l * np.sin(state[0]) -
                self.b * state[1] -
                np.sign(state[1])*self.coulomb_fric) / self.inertia
        return accn

    def rhs(self, t, state, tau):

        if isinstance(tau, (list, tuple, np.ndarray)):
            torque = tau[0]
        else:
            torque = tau

        # Forward dynamics
        accn = self.forward_dynamics(state, torque)

        # Next state
        res = np.zeros(2*self.dof)
        res[0] = state[1]
        res[1] = accn
        return res
