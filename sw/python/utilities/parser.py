# global imports
import argparse
from argparse import RawTextHelpFormatter


def syntax_parser():
    parser = argparse.ArgumentParser(
        description='''                     Control of a torque limited Simple Pendulum

                            Underactuated LAB at DFKI Bremen
        ''', formatter_class=RawTextHelpFormatter)
#    parser.add_argument("-sim", action='store_true',
#                        help="simulate the simple pendulum plant, instead "
#                             "of controlling the real system")
    actuator = parser.add_mutually_exclusive_group(required=False)
    actuator.add_argument("-qdd100", action='store_true',
                          help="to control the qdd100 actuators from mjbots")
    actuator.add_argument("-ak80_6", action='store_true',
                          help="to control the AK80-6 actuators from t-motors")
    controller = parser.add_mutually_exclusive_group(required=True)
    controller.add_argument("-pd", action='store_true',
                            help="proportional derivative control mode")
    controller.add_argument("-fftau", action='store_true',
                            help="feedforward torque control mode")
    controller.add_argument("-gravity", action='store_true',
                            help="gravity compensation mode")
    controller.add_argument("-lqr", action='store_true',
                            help="linear quadratic regulator")
    controller.add_argument("-ilqr", action='store_true',
                            help="iterative linear quadratic regulator")
    controller.add_argument("-energy", action='store_true',
                            help="linear quadratic regulator")
    controller.add_argument("-sac", action='store_true',
                            help="soft actor critic")
    controller.add_argument("-ddp", action='store_true',
                            help="differential dynamic programming")
    parser.add_argument("-save", action='store_true',
                        help="saves your measurements into (../results)",
                        required=False)

    # parsing
    args, unknown = parser.parse_known_args()

    return args, unknown
