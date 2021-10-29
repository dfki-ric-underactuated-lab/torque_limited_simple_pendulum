from motor_driver.canmotorlib import CanMotorController

from simple_pendulum.utilities.performance_profiler import motor_speed_test


# Motor ID
motor_id = 0x01
can_port = 'can0'
motor_1 = CanMotorController(can_port, motor_id)

motor_speed_test(motor_1, n=1000)
