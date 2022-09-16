# from https://stackoverflow.com/questions/46506850/how-can-i-get-input-from-an-xbox-one-controller-in-python

from inputs import get_gamepad
import threading


class GamePad(object):

    def __init__(self, gamepad_name="Logitech Logitech RumblePad 2 USB"):

        self.MAX_JOY_VAL = 255.  # math.pow(2, 15)

        self.LeftJoystickX = 0

        self._monitor_thread = threading.Thread(target=self._monitor_controller, args=())
        self._monitor_thread.daemon = True
        self._monitor_thread.start()

    def read(self):
        x = self.LeftJoystickX
        return x

    def _monitor_controller(self):
        while True:
            events = get_gamepad()
            for event in events:
                if event.code == 'ABS_X':
                    self.LeftJoystickX = 2*(event.state / self.MAX_JOY_VAL - 0.5)  # normalize between -1 and 1


if __name__ == '__main__':
    joy = GamePad()
    while True:
        print(joy.read())
