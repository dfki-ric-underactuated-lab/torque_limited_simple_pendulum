# here, the abstract controller class is defined, to which all controller
# classes have to adhere
from abc import ABC, abstractmethod


class AbstractController(ABC):
    @abstractmethod
    def get_control_output(self, meas_pos, meas_vel, meas_tau, meas_time):
        des_pos = None
        des_vel = None
        des_tau = None
        return des_pos, des_vel, des_tau
