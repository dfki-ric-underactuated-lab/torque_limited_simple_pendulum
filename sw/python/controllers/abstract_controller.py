# here, the abstract controller class is defined, to which all controller classes have to adhere

from abc import ABC, abstractmethod


class AbstractController(ABC):
    @abstractmethod
    def get_control_output(self, pos, vel, tau, t, i):
        pos_des = None
        vel_des = None
        tau_des = None

        return pos_des, vel_des, tau_des
