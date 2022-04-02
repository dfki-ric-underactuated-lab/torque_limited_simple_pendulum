"""
Parsing
=======
"""


# global imports
import argparse
from argparse import RawTextHelpFormatter


def syntax():
    parser = argparse.ArgumentParser(
        description='''                      Control of a real torque-limited Simple Pendulum

                            Underactuated LAB at DFKI Bremen


        Besides the distinction in model-based controllers (gravity compensation, 
        energy shaping, fddp, lqr, ilqr) and data-driven controllers (sac, ddpg) 
        the control methods are also divided into open and closed loop control. 
        The former execute a precomputed trajectory, which is derived from one 
        out of two trajectory optimization techniques:

            - Direct Collocation
            - Feasibility-Driven Dynamic Programming

        The latter don't rely on a precomputed trajectory, but generate their 
        input torques online based on the implemented control policy. You can 
        choose from a variety of different control methods by adding the flag 
        of your desired controller, e.g. '-lqr' or '-energy', to 'main.py'. In 
        order to use openloop controllers you additionally need to specify the 
        '-openloop' flag, for example to run the real pendulum with 
        Feasibility-Driven Dynamic Programming the corresponding terminal 
        command would be 'python main.py -openloop -fddp'. We further provide 
        the option to save all measurements and plots in the results folder by 
        adding the -save flag.
        ''', formatter_class=RawTextHelpFormatter)
    controller = parser.add_mutually_exclusive_group(required=True)
    controller.add_argument("-gravity", action='store_true',
                            help="gravity compensation mode")
    controller.add_argument("-lqr", action='store_true',
                            help="linear quadratic regulator")
    controller.add_argument("-ilqrmpc", action='store_true',
                            help="model predictive control with" + \
                                  "iterative linear quadratic regulator")
    controller.add_argument("-energy", action='store_true',
                            help="energy shaping + linear quadratic regulator")
    controller.add_argument("-sac", action='store_true',
                            help="soft actor critic")
    controller.add_argument("-ddpg", action='store_true',
                            help="deep deterministic policy gradient")
    controller.add_argument("-ddp", action='store_true',
                            help="ddp trajectory, " + \
                                  "choose -fft (feed-forward torque, default)," + \
                                  "-pid (proportional integral derivative) or" + \
                                  "-tvlqr (time-varying LQR)")
    controller.add_argument("-dircol", action='store_true',
                            help="direct collocation trajectory," + \
                                  "choose -fft (feed-forward torque, default)," + \
                                  "-pid (proportional integral derivative) or" + \
                                  "-tvlqr (time-varying LQR)")
    controller.add_argument("-ilqr", action='store_true',
                            help="ilqr trajectory," + \
                                  "choose -fft (feed-forward torque, default)," + \
                                  "-pid (proportional integral derivative) or" + \
                                  "-tvlqr (time-varying LQR)")

    trajectory = parser.add_mutually_exclusive_group(required=False)
    trajectory.add_argument("-fft", action='store_true',
                          help="feedforward torque control")
    trajectory.add_argument("-tvlqr", action='store_true',
                          help="time-varying LQR")
    trajectory.add_argument("-pd", action='store_true',
                          help="proportional derivative motor control mode")

    parser.add_argument("-save", action='store_true',
                        help="saves your measurements into (../results)",
                        required=False)

    # parsing
    args, unknown = parser.parse_known_args()

    return args, unknown
