
def actuator():
    motor_id = 0x03
    gr = 1                          # the gear ratio
    Kp = 50                         # proportional gains
    Kd = 4                          # derivative gains
    return motor_id, gr, Kp, Kd


def robot(URDF_FILE):
    joints = 2
