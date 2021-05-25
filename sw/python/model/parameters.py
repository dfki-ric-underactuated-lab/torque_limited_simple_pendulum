
def actuator():
    motor_id = 0x03
    gr = 1                          # the gear ratio
    kp = 50                         # proportional gains
    kd = 4                          # derivative gains
    return motor_id, gr, kp, kd


def robot():
