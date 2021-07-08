# global imports
import argparse
from argparse import RawTextHelpFormatter


def syntax():
    parser = argparse.ArgumentParser(
        description='''                     Control of a torque limited Simple Pendulum

                            Underactuated LAB at DFKI Bremen
        ''', formatter_class=RawTextHelpFormatter)
    controller = parser.add_mutually_exclusive_group(required=True)
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
    controller.add_argument("-openloop", action='store_true',
                            help="open loop control mode, choose either -pd "
                                 "or -ftt ")
    openloop = parser.add_mutually_exclusive_group(required=False)
    openloop.add_argument("-pd", action='store_true',
                          help="proportional derivative control mode")
    openloop.add_argument("-fft", action='store_true',
                          help="feedforward torque control mode")
    parser.add_argument("-save", action='store_true',
                        help="saves your measurements into (../results)",
                        required=False)

    # parsing
    args, unknown = parser.parse_known_args()

    return args, unknown
