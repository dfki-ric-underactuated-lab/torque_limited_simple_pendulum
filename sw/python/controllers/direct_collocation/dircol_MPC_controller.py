from controllers.direct_collocation.direct_collocation import DirectCollocationCalculator


class DirectCollocationMPC():
    def __init__(self, mass=1.0, length=0.5, damping=0.1,
                 gravity=9.81, torque_limit=2.0):
        self.dircal = DirectCollocationCalculator()
        self.dircal.init_pendulum(mass=mass,
                                  length=length,
                                  damping=damping,
                                  gravity=gravity,
                                  torque_limit=torque_limit)

        self.goal = [0.0, 0.0]
        self.N_collocation_points = 21
        self.max_dt = 0.5
        self.N_sampling_points = 400

        self.x_traj = None

    def set_goal(self, x):
        self.goal = x  # theta, theta_dot

    def get_control_output(self, x):
        self.x_traj, dircol, result = self.dircal.compute_trajectory(
                                              N=self.N_collocation_points,
                                              max_dt=self.max_dt,
                                              start_state=x,
                                              goal_state=self.goal,
                                              initial_x_trajectory=self.x_traj)
        T, X, XD, U = self.dircal.extract_trajectory(self.x_traj,
                                                     dircol,
                                                     result,
                                                     N=self.N_sampling_points)
        return U[0]
