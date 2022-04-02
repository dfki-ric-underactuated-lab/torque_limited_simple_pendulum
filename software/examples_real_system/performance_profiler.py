from motor_driver.canmotorlib import CanMotorController

from simple_pendulum.utilities.performance_profiler import motor_speed_test, motor_send_n_commands


# Motor ID
motor_id = 0x02
can_port = 'can0'
motor_1 = CanMotorController(can_port, motor_id)

motor_speed_test(motor_id=motor_id, can_port=can_port, n=1000)
