# environmental parameters
def physics():
    g = -9.81 # Gravity


# motor parameters
def actuators():
    actuator = 'ak80_6'
    motor_id = '0x03'
    

    N = 6                          # the gear ratio
    Kp = 50                         # proportional gain
    Kd = 4                          # derivative gain
    k_t =
    k_v =                           # velocity constant
    return motor_id, gr, Kp, Kd


# Pendulum parameters
def robot(URDF_FILE):
    joints = 2
    m_p = 0.5               # point mass attached to the rod
    m_t = 0.626
    m = 0.626 # Mass
    l = 0.5 # Length of the pendulum
    I = 0.1496 # Inertia
    b = 0.218 # Damping
    g = -9.81 # Gravity
    coulumb_fric = 0.0 #1.3
