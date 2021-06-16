from datetime import datetime
import argparse
from argparse import RawTextHelpFormatter

def SyntaxParser(WORK_DIR):
    parser = argparse.ArgumentParser(
        description='''                     Control of a torque limited Simple Pendulum

                            Underactuated LAB at DFKI Bremen
        ''', formatter_class=RawTextHelpFormatter)
    parser.add_argument("-pd", action='store_true',
                        help="position and velocity control mode", required=False)
    parser.add_argument("-tau", action='store_true',
                        help="torque control mode")
    parser.add_argument("-lqr", action='store_true',
                        help="linear quadratic regulator", required=False)
    parser.add_argument("-ddp", action='store_true',
                        help="differential dynamic programming", required=False)
    parser.add_argument("-sim", action='store_true',
                        help="simulate the simple pendulum plant, instead of controlling the real system",  required=False)
    parser.add_argument("-qdd100", action='store_true',
                        help="to control the qdd100 actuators from mjbots",  required=False)
    parser.add_argument("-ak80_6", action='store_true',
                        help="to control the AK80-6 actuators from t-motors", required=False)
    parser.add_argument("-save", action='store_true',
                        help="saves your measurements into (../results)", required=False)

    # Execute the parse_args method
    args, unknown = parser.parse_known_args()    

    folder_name = ""
    if args.pd:
        folder_name = "pd_control"
    if args.tau:
        folder_name = "torque_control"
    if args.lqr:
        folder_name = "lqr"
    if args.ddp:
        folder_name = "ddp"

    TIMESTAMP = datetime.now().strftime(                # get timestamp              
        "%Y%m%d-%I%M%S-%p")
    OUTPUT_FOLDER = str(WORK_DIR) + f'/results/{TIMESTAMP}_' + folder_name

    return OUTPUT_FOLDER, args, unknown