# from https://stackoverflow.com/questions/46506850/how-can-i-get-input-from-an-xbox-one-controller-in-python

#from inputs import get_gamepad
from evdev import ecodes, InputDevice, ff, util
import threading


# class GamePad(object):
#
#     def __init__(self, gamepad_name="Logitech Logitech RumblePad 2 USB"):
#
#         self.MAX_JOY_VAL = 255.  # math.pow(2, 15)
#
#         self.LeftJoystickX = 0
#         self.LeftBumper = 0
#         self.RightBumper = 0
#
#         self._monitor_thread = threading.Thread(target=self._monitor_controller, args=())
#         self._monitor_thread.daemon = True
#         self._monitor_thread.start()
#
#         if gamepad_name == "Logitech Logitech RumblePad 2 USB":
#             self.left_stick_x_name = "ABS_X"
#             self.left_bumper_name = "BTN_TOP2"
#             self.right_bumper_name = "BTN_PINKIE"
#         elif gamepad_name == "Logitech WingMan Cordless Gamepad":
#             self.left_stick_x_name = "ABS_RZ"
#             self.left_bumper_name = "BTN_BASE"
#             self.right_bumper_name = "BTN_BASE2"
#
#     def read(self):
#         x = self.LeftJoystickX
#         lb = self.LeftBumper
#         rb = self.RightBumper
#         return x, lb, rb
#
#     def _monitor_controller(self):
#         while True:
#             events = get_gamepad()
#             for event in events:
#                 if event.code == self.left_stick_x_name:
#                     self.LeftJoystickX = 2*(event.state / self.MAX_JOY_VAL - 0.5)  # normalize between -1 and 1
#                 elif event.code == self.left_bumper_name:
#                     self.LeftBumper = event.state
#                 elif event.code == self.right_bumper_name:
#                     self.RightBumper = event.state
#     def rumble(self):
#         pass
#
#     def stop_rumble(self):
#         pass

class GamePad(object):

    def __init__(self, gamepad_name="Logitech Logitech RumblePad 2 USB", dt=0.005):

        self.MAX_JOY_VAL = 255.  # math.pow(2, 15)

        self.LeftJoystickX = 0
        self.LeftBumper = 0
        self.RightBumper = 0

        self._monitor_thread = threading.Thread(target=self._monitor_controller, args=())
        self._monitor_thread.daemon = True
        self._monitor_thread.start()

        self.gamepad_name = gamepad_name

        if gamepad_name == "Logitech Logitech RumblePad 2 USB":
            self.left_stick_x_name = ecodes.ABS_X
            self.left_bumper_name = ecodes.BTN_TOP2
            self.right_bumper_name = ecodes.BTN_PINKIE
        elif gamepad_name == "Logitech WingMan Cordless Gamepad":
            self.left_stick_x_name = ecodes.ABS_RZ
            self.left_bumper_name = ecodes.BTN_BASE
            self.right_bumper_name = ecodes.BTN_BASE2

        # get input device
        for name in util.list_devices():
            self.dev2 = InputDevice(name)
            if self.dev2.name == self.gamepad_name:
                break

        rumble_effect = ff.Rumble(strong_magnitude=65535, weak_magnitude=65535)
        effect_type = ff.EffectType(ff_rumble_effect=rumble_effect)
        duration_ms = int(dt*1000)

        self.effect = ff.Effect(
            ecodes.FF_RUMBLE, # type
            -1, # id (set by ioctl)
            0,  # direction
            ff.Trigger(0, 0), # no triggers
            ff.Replay(duration_ms, 0), # length and delay
            ff.EffectType(ff_rumble_effect=rumble_effect)
        )

        self.rumble_on = False

    def read(self):
        x = self.LeftJoystickX
        lb = self.LeftBumper
        rb = self.RightBumper
        return x, lb, rb

    def _monitor_controller(self):
        while True:
            # get input device
            for name in util.list_devices():
                self.dev = InputDevice(name)
                if self.dev.name == self.gamepad_name:
                    break
            # read inputs
            for event in self.dev.read_loop():
                if event.type == ecodes.EV_ABS:
                    if event.code == self.left_stick_x_name:
                        self.LeftJoystickX = 2*(event.value / self.MAX_JOY_VAL - 0.5)
                elif event.type == ecodes.EV_KEY:
                    if event.code == self.left_bumper_name:
                        self.LeftBumper = event.value
                    if event.code == self.right_bumper_name:
                        self.RightBumper = event.value

    def rumble(self):
        self.effect_id = self.dev2.upload_effect(self.effect)
        self.dev2.write(ecodes.EV_FF, self.effect_id, 1)
        self.rumble_on = True

    def stop_rumble(self):
        if self.rumble_on:
            self.dev2.erase_effect(self.effect_id)
            self.rumble_on = False

if __name__ == '__main__':
    joy = GamePad()
    while True:
        print(joy.read())
