from simple_pendulum.controllers.abstract_controller import AbstractController


class CombinedController(AbstractController):
    """
    Controller to combine two controllers and switch between them on conditions

    Parameters
    ----------
    controller1 : Controller object
        First Controller
    controller2 : Controller object
        Second Controller
    condition1 : function of (meas_pos, meas_vel, meas_tau, meas_time)
        condition to switch to controller 1
        must be a functin of the state x and the time t
    condition2 : function of (meas_pos, meas_vel, meas_tau, meas_time)
        condition to switch to controller 2
        must be a functin of the state x and the time t
    compute_both : bool
        Flag whether to compute the control output for both controllers at each
        timestep or only for the active one
    """
    def __init__(self,
                 controller1,
                 controller2,
                 condition1,
                 condition2,
                 compute_both=False):

        super().__init__()

        self.controllers = [controller1, controller2]
        self.active = 0

        self.conditions = [condition1, condition2]

        self.compute_both = compute_both

    def init(self, x0):
        """
        initialize both controllers
        """
        self.controllers[0].init(x0)
        self.controllers[1].init(x0)

    def set_goal(self, x):
        """
        Set the desired state for the controllers.

        Parameters
        ----------
        x : array like
            the desired goal state of the controllers
        """
        self.controllers[0].set_goal(x)
        self.controllers[1].set_goal(x)

    def get_control_output(self, meas_pos, meas_vel, meas_tau, meas_time):
        """
        The function to compute the control input for the pendulum actuator.
        Supposed to be overwritten by actual controllers. The API of this
        method should be adapted. Unused inputs/outputs can be set to None.

        Parameters
        ----------

        meas_pos: float
            The position of the pendulum [rad]
        meas_vel: float
            The velocity of the pendulum [rad/s]
        meas_tau: float
            The meastured torque of the pendulum [Nm]
        meas_time: float
            The collapsed time [s]

        Returns
        -------
        des_pos: float
            The desired position of the pendulum [rad]
        des_vel: float
            The desired velocity of the pendulum [rad/s]
        des_tau: float
            The torque supposed to be applied by the actuator [Nm]
        """
        inactive = 1 - self.active

        if self.conditions[inactive](meas_pos, meas_vel, meas_tau, meas_time):
            self.active = 1 - self.active
            print("Switching to Controller ", self.active + 1)

        if self.compute_both:
            _ = self.controllers[inactive].get_control_output(meas_pos, meas_vel, meas_tau, meas_time)

        return self.controllers[self.active].get_control_output(meas_pos, meas_vel, meas_tau, meas_time)

