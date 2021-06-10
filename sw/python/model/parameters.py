import math

################################################
# Environmental parameters
################################################
class environment:
    def __init__(self, g):
        self.g = g                              # Gravity constant


# define environments
earth = environment()
earth.g = -9.81                                 # [m/s**2]


################################################
# Robot parameters
################################################
class robot:
    def __init__(self, base, mass, joints,
                 links, dof):
        self.base = base                        # Fixed, floating, ....
        self.mass = mass                        # Overall mass
        self.joints = joints                    # Number of joints
        self.links = links                      # Number of links
        self.dof = dof                          # Degrees of freedom


# define robots
sp = robot()                                    # Simple Pendulum
sp.base = "fixed"
sp.mass = None
sp.joints = 1
sp.links = 1
sp.dof = 1


################################################
# Joint parameters
################################################
class joint:
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
        self.fv = fv                            # Viskose friction
        self.b = b                              # Damping


# define joints
joint_001 = joint()
joint_001.num = 1
joint_001.type = "revolute"
joint_001.dof = 1
joint_001.dof_t = 0
joint_001.dof_r = 1
joint_001.tx = 0
joint_001.ty = 0
joint_001.tz = 0
joint_001.rx = 0
joint_001.ry = 0
joint_001.rz = 1
joint_001.fc = 1.3                              # add unit
joint_001.fv = 0                                # add unit
joint_001.b = 0.218                             # add unit


################################################
# Link parameters
################################################
class link:
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
        self.inertia = (mass_l*length**2)/3     # inertia considering the CoM
                    + mass_p*length**2
        #self.inertia = (m_p+m_l)*l**2          # inertia assuming a point mass
                                                # at the end of the rod


# define links
link_001 = link()
link_001.num = 1
link_001.mass = 0.6755                          # [kg]
link_001.mass_p = 0.5                           # [kg]
link_001.mass_l = link_001.mass                 # [kg]
                    - link_001.mass_p
link_001.length = 0.5                           # [m]


################################################
# Motor parameters
################################################
class actuator:
    def __init__(self, can_id, poles, wiring,
                 r, v_max, a_max, a_rated, mass,
                 tau_max, tau_rated, vel_max,
                 gr, inertia, resist, induct,
                 kp, kd, k_m, k_v, v_per_hz,):
        self.can_id = can_id
        self.mass = mass
        self.v_max = v_max                      # Max. voltage
        self.a_max = a_max                      # Max. current
        self.a_rated = a_rated
        self.tau_max = tau_max                  # Max. torque
        self.tau_rated = tau_rated
        self.vel_max = vel_max                  # Max. velocity at 24V
        self.gr = gr                            # Gear ratio
        self.poles = poles                      # Number of poles
        self.wiring = wiring                    # Motor wiring
        self.inertia = inertia                  # Rotor inertia
        self.resist = resist                    # Resistance (phase to phase)
        self.induct = induct                    # Inductancee (phase to phase)
        self.v_per_hz = v_per_hz                # Voltage per hz

        # motor constants
        self.k_m = k_m                          # motor constant
        self.k_v = k_v                          # velocity / backEMF constant
        self.k_e = 1/k_v                        # elctrical constant
        self.k_t = k_m*math.sqrt(r)             # torque constant

        # Controller variables
        #self.kd = kd                            # Proportional gain
        #self.kp = kp                            # Derivative gain


# define actuators
qdd100_001 = actuator()
qdd100_001.can_id = 1
qdd100_001.mass = 0.485                         # [kg]
qdd100_001.poles = 42
qdd100_001.gr = 1/6
qdd100_001.v_max = 24                           # Volts
qdd100_001.a_max = 24                           # Ampere
qdd100_001.a_rated = 12                         # Ampere
qdd100_001.tau_max = 12                         # Nm    (after transmission)
qdd100_001.tau_rated = 6                        # Nm    (after transmission)
qdd100_001.vel_max = 38.2                       # rad/s (after transmission)

qdd100_001.v_per_hz = None
qdd100_001.r = r                                                #
qdd100_001.kd = kd                              # Proportional gain
qdd100_001.kp = kp                              # Derivative gain

ak80_6_001 = actuator()



def actuator():
    actuator = 'ak80_6'
    motor_id = '0x03'
    N = 6                                       # the gear ratio
    Kp = 50                                     # proportional gain
    Kd = 4                                      # derivative gain
    k_t =
    k_v =                                       # velocity constant
    return motor_id, gr, Kp, Kd
