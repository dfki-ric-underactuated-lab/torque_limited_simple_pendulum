"""
Parameters
==========
"""


# Global imports
import math
import yaml
import numpy as np


def get_params(params_path):
    """
    Retrieve parameters from a yaml file with name ``params_path``
    """
    with open(params_path, 'r') as fle:
        params = yaml.safe_load(fle)
    return params


class Environment:
    def __init__(self):
        """
        Environmental parameters
        """
        self.gravity = None                             # Gravity constant


# define environments
earth = Environment()
earth.gravity = 9.81                                    # [m/s^2]


class Robot:
    def __init__(self):
        """
        Robot parameters
        """
        self.base = None                                # Fixed, floating, ....
        self.origin = None
        self.mass = None                                # Overall mass
        self.n_joints = None                            # Number of joints
        self.n_actuators = None                         # Number of links
        self.n_links = None                             # Number of links
        self.dof = None                                 # Degrees of freedom


# define robots
sp = Robot()                                            # Simple Pendulum
sp.base = "fixed"
sp.origin = [0, 0]
sp.mass = None
sp.n_joints = 1
sp.n_links = 1
sp.n_actuators = 1
sp.dof = 1


class Joints:
    def __init__(self):
        """
        Joint parameters
        """
        self.num = None                         # Number in the kinematic chain
        self.type = None                        # Type: prismatic, revolute,
                                                # spherical, cylindrical,
                                                # cartesian, ...
        self.dof = None
        self.dof_t = None                       # Translational DoF
        self.dof_r = None                       # Rotational DoF
        self.tx = None
        self.ty = None
        self.tz = None
        self.rx = None
        self.ry = None
        self.rz = None
        self.fc = None                          # Coulomb friction
        self.fv = None                          # Viscous friction
        self.b = None                           # Damping


joint_01 = Joints()                             # define joints
joint_01.num = 1
joint_01.type = "revolute"
joint_01.dof = 1
joint_01.dof_t = 0
joint_01.dof_r = 1
joint_01.tx = 0
joint_01.ty = 0
joint_01.tz = 0
joint_01.rx = 0
joint_01.ry = 0
joint_01.rz = 1
joint_01.fc = 1.3                               # N
joint_01.fv = 0                                 # N
joint_01.b = 0.1                                # add unit


class Links:
    def __init__(self):
        """
        Link parameters
        """
        self.num = None                         # Number within kinematic chain
        self.mass = None                        # Overall mass
        self.mass_p = None                      # Point mass at link end
        self.mass_l = None                      # Link mass without point mass
        self.length = None                      # Length between joints
        self.inertia = None                     # Inertia assuming a point
                                                # mass at the end of the rod
        self.length_CoM = None                  # From preceding joint to CoM
        self.inertia_CoM = None                 # Inertia considering the CoM

    def calc_m_l(self, mass, mass_p):
        self.mass_l = mass - mass_p
        return self.mass_l

    def calc_length_com(self, mass_p, mass_l, length):
        self.length_CoM = (mass_p*length + 0.5 * mass_l * length) / \
                          (mass_p + mass_l)
        return self.length_CoM

    def calc_inertia(self, mass_p, mass_l, length):
        self.inertia = (mass_p + mass_l) * length**2
        return self.inertia

    def calc_inertia_com(self, mass_p, mass_l, length):
        self.inertia_CoM = (mass_l * length**2) / 3 + (mass_p * length**2)
        return self.inertia_CoM


link_01 = Links()                               # define links
link_01.num = 1
link_01.mass = 0.6755                           # [kg]
link_01.mass_p = 0.5                            # [kg]
link_01.mass_l = 0.1755                         # [kg]
link_01.length = 0.5                            # [m]


class Actuators:
    def __init__(self):
        """
        Motor parameters
        """
        self.can_id = None
        self.mass = None
        self.v_max = None                       # Max. voltage
        self.a_max = None                       # Max. current
        self.a_rated = None
        self.gr = None                          # Gear ratio
        self.backlash = None
        self.tau_max = None                     # Max. torque
        self.tau_rated = None
        self.vel_max = None                     # Max. velocity at 24V
        self.poles = None                       # Number of poles
        self.wiring = None                      # Motor wiring (delta, star...)
        self.resist = None                      # Resistance (phase to phase)
        self.induct = None                      # Inductance (phase to phase)
        self.v_per_hz = None                    # Voltage per hz
        self.inertia = None                     # Rotor inertia
        # Motor constants
        self.k_m = None                         # Motor constant
        self.k_v = None                         # Velocity / backEMF constant
        self.k_e = None                         # Electrical constant
        self.k_t = None                         # Torque constant
        # Controller variables
        self.kd = None                          # Proportional gain
        self.kp = None                          # Derivative gain

    def calc_k_m(self, k_t, resist):
        self.k_m = k_t / np.sqrt(resist)
        return self.k_m

    def calc_k_v(self, v_per_hz):
        self.k_v = 0.5 * 60 / v_per_hz
        return self.k_v

    def calc_k_e(self, k_v):
        self.k_e = 1/k_v
        return self.k_e

    def calc_k_t_from_k_m(self, k_m, resist):
        self.k_t = k_m*math.sqrt(resist)
        return self.k_t

    def calc_k_t_from_k_v(self, k_v):
        self.k_t = 0.78 * 60 / (2 * np.pi * k_v)
        return self.k_t


ak80_6_01 = Actuators()                         # From t-motor
ak80_6_01.can_id = '0x01'
ak80_6_01.mass = 0.485                          # [kg]
ak80_6_01.v_max = 24                            # Volts
ak80_6_01.a_max = 24                            # [A]
ak80_6_01.a_rated = 12                          # [A]
ak80_6_01.gr = 1/6
ak80_6_01.backlash = 0.15                       # Degree  
ak80_6_01.tau_max = 12                          # Nm    (after transmission)
ak80_6_01.tau_rated = 6                         # Nm    (after transmission)
ak80_6_01.vel_max = 38.2                        # rad/s (after transmission)
ak80_6_01.inertia = 60.719e-6                   # [kg*m^2]
ak80_6_01.poles = 42
ak80_6_01.wiring = "delta"
ak80_6_01.resist = 170e-3                       # Ohm
ak80_6_01.induct = 57e-6                        # Henry
ak80_6_01.v_per_hz = None                       # Vs
ak80_6_01.k_v = 100                             # rpm/V
ak80_6_01.k_m = 0.2206                          # Nm/sqrt(W) 
ak80_6_01.kp = 50                              
ak80_6_01.kd = 4                             

qdd100_01 = Actuators()                         # From mjbots
qdd100_01.can_id = '0x01'
qdd100_01.mass = 0.485                          # [kg]
qdd100_01.v_max = 44                            # Volts
qdd100_01.a_max = 12                            # [A]
qdd100_01.a_rated = 6                           # [A]
qdd100_01.gr = 1/6
qdd100_01.backlash = 0.1                        # Degree
qdd100_01.tau_max = 16                          # Nm    (after transmission)
qdd100_01.tau_rated = 6                         # Nm    (after transmission)
qdd100_01.vel_max = 64.6                        # rad/s (after transmission)
qdd100_01.inertia = None                        # [kg*m^2]
qdd100_01.poles = 42
qdd100_01.wiring = None
qdd100_01.resist = 64e-3                        # Ohm     
qdd100_01.induct = 48.6e-06                     # Henry                     
qdd100_01.v_per_hz = 0.2269                     # Vs
qdd100_01.k_v = 132.18                          # rpm/V
qdd100_01.k_m = 0.2227                          # Nm/sqrt(W)
qdd100_01.kp = 100                              
qdd100_01.kd = 2
