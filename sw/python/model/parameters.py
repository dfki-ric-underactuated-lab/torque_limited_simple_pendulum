# Global imports
import math
import yaml


class Environment:
    """
    Environmental parameters
    """
    def __init__(self, gravity):
        self.gravity = gravity                  # Gravity constant


# define environments
earth = Environment()
earth.gravity = 9.81                            # [m/s^2]


class Robot:
    """
    Robot parameters
    """
    def __init__(self, base, origin, mass, n_joints,
                 n_links, n_actuators, dof):
        self.base = base                        # Fixed, floating, ....
        self.origin = origin
        self.mass = mass                        # Overall mass
        self.n_joints = n_joints                # Number of joints
        self.n_actuators = n_actuators          # Number of links
        self.n_links = n_links                  # Number of links
        self.dof = dof                          # Degrees of freedom


# define robots
sp = Robot()                                    # Simple Pendulum
sp.base = "fixed"
sp.origin = [0, 0]
sp.mass = None
sp.n_joints = 1
sp.n_links = 1
sp.n_actuators = 1
sp.dof = 1


class Joints:
    """
    Joint parameters
    """
    def __init__(self, num, type, dof, dof_t,
                 dof_r, tx, ty, tz, rx, ry, rz,
                 fc, fv, b):
        self.num = num                          # Number in the kinematic chain
        self.type = type                        # Type: prismatic, revolute,
                                                # spherical, cylindrical,
                                                # cartesian, ...
        self.dof = dof
        self.dof_t = dof_t                      # Translational DoF
        self.dof_r = dof_r                      # Rotational DoF
        self.tx = tx
        self.ty = ty
        self.tz = tz
        self.rx = rx
        self.ry = ry
        self.rz = rz
        self.fc = fc                            # Coulomb friction
        self.fv = fv                            # Viscous friction
        self.b = b                              # Damping


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
    """
    Link parameters
    """
    def __init__(self, num, mass, mass_p,
                 mass_l, length):
        self.num = num                          # Number within kinematic chain
        self.mass = mass                        # Overall mass
        self.mass_p = mass_p                    # Point mass at link end
        self.mass_l = mass_l                    # Link mass without point mass
        self.length = length                    # Length between joints
        self.length_com = (mass_p*length        # From preceding joint to CoM
                    + 0.5*mass_l*length)/(mass_p
                    + mass_l)
        self.inertia = ((mass_l*length**2)/3    # Inertia considering the CoM
                    + mass_p*length**2)
        # self.inertia = (m_p+m_l)*l**2         # Inertia assuming a point
                                                # mass at the end of the rod


link_01 = Links()                               # define links
link_01.num = 1
link_01.mass = 0.6755                           # [kg]
link_01.mass_p = 0.5                            # [kg]
link_01.mass_l = (link_01.mass                  # [kg]
                  - link_01.mass_p)
link_01.length = 0.5                            # [m]


class Actuators:
    """
    Motor parameters
    """
    def __init__(self, can_id, poles, wiring,
                 r, v_max, a_max, a_rated, mass,
                 tau_max, tau_rated, vel_max,
                 gr, backlash, inertia, resist, 
                 kp, kd, k_m, k_v, induct,
                 v_per_hz,):
        self.can_id = can_id
        self.mass = mass
        self.v_max = v_max                      # Max. voltage
        self.a_max = a_max                      # Max. current
        self.a_rated = a_rated
        self.gr = gr                            # Gear ratio
        self.backlash = backlash                
        self.tau_max = tau_max                  # Max. torque
        self.tau_rated = tau_rated
        self.vel_max = vel_max                  # Max. velocity at 24V
        self.poles = poles                      # Number of poles
        self.wiring = wiring                    # Motor wiring (delta, star...)
        self.resist = resist                    # Resistance (phase to phase)
        self.induct = induct                    # Inductance (phase to phase)
        self.v_per_hz = v_per_hz                # Voltage per hz
        self.inertia = inertia                  # Rotor inertia
        # Motor constants
        self.k_m = k_m                          # Motor constant
        self.k_v = k_v                          # Velocity / backEMF constant
        self.k_e = 1/k_v                        # Electrical constant
        self.k_t = k_m*math.sqrt(resist)        # Torque constant
        # Controller variables
        self.kd = kd                            # Proportional gain
        self.kp = kp                            # Derivative gain


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
ak80_6_01.v_per_hz = None                      
ak80_6_01.k_v = 100                             # rpm/V
ak80_6_01.k_m = 0.2206                          # Nm/sqrt(W) 
ak80_6_01.kp = 50                              
ak80_6_01.kd = 4                             

qdd100_01 = Actuators()                          # From mjbots
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
qdd100_01.v_per_hz = 0.2269                    
# k_v = 0.5 * 60 / motor.v_per_hz
qdd100_01.k_v = 132.18                          # rpm/V
# k_t = 0.78 * 60 / (2 * pi * k_v) = 0.056  
# k_m = k_t / sqrt(resist) = 0.2227
qdd100_01.k_m = 0.2227                          # Nm/sqrt(W)
qdd100_01.kp = 100                              
qdd100_01.kd = 2


def get_params(params_path):
    with open(params_path, 'r') as yaml_file:
        params = yaml.safe_load(yaml_file)
#        gravity = params["gravity"]
#        mass = params["mass"]
#        inertia = params["inertia"]
#        base = params["base"]
#        dof = params["dof"]
#        n_links = params["n_links"]
#        length = params["length"]
#        n_actuators = params["n_actuators"]
#        torque_limit = params["torque_limit"]
#        gr = params["gr"]
#        damping = params["damping"]
#        coulomb_fric = params["coulomb_fric"]
#        parameters = [gravity, mass, inertia, base, dof, n_links,
#                      length, n_actuators, torque_limit, gr, damping,
#                      coulomb_fric]
        return params
