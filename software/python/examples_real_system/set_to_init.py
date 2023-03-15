from motor_driver.canmotorlib import CanMotorController
import time

motor = CanMotorController(can_socket='can0', motor_id=0x01, motor_type="AK80_6_V2", socket_timeout=0.5)

motor.enable_motor()

motor.send_deg_command(0, 10, 0, 2, 0.1)
time.sleep(1)
motor.send_deg_command(0, 0, 0, 0, 0)

motor.disable_motor()
