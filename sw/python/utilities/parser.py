# global imports
import argparse
from argparse import RawTextHelpFormatter


def syntax_parser():
    parser = argparse.ArgumentParser(
        description='''                     Control of a torque limited Simple Pendulum

                            Underactuated LAB at DFKI Bremen
        ''', formatter_class=RawTextHelpFormatter)
    parser.add_argument("-pd", action='store_true',
                        help="proportional derivative control mode",
                        required=False)
    parser.add_argument("-tau", action='store_true',
                        help="torque control mode")
    parser.add_argument("-lqr", action='store_true',
                        help="linear quadratic regulator", required=False)
    parser.add_argument("-ilqr", action='store_true',
                        help="iterative linear quadratic regulator",
                        required=False)
    parser.add_argument("-energy", action='store_true',
                        help="linear quadratic regulator", required=False)
    parser.add_argument("-sac", action='store_true',
                        help="soft actor critic",
                        required=False)
    parser.add_argument("-ddp", action='store_true',
                        help="differential dynamic programming",
                        required=False)
    parser.add_argument("-sim", action='store_true',
                        help="simulate the simple pendulum plant, instead of "
                             "controlling the real system",  required=False)
    parser.add_argument("-save", action='store_true',
                        help="saves your measurements into (../results)",
                        required=False)
    # parser.add_argument("-qdd100", action='store_true',
    #                    help="to control the qdd100 actuators from mjbots",
    #                    required=False)
    # parser.add_argument("-ak80_6", action='store_true',
    #                    help="to control the AK80-6 actuators from t-motors",
    #                    required=False)

    # parsing
    args, unknown = parser.parse_known_args()

    return args, unknown
