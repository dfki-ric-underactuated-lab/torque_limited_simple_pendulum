"""
Units
=====
"""


# global imports
import numpy as np


def rad_to_deg(theta_rad):
    theta_deg = 180.0*np.pi
    return theta_deg


def deg_to_rad(theta_deg):
    theta_rad = (theta_deg*np.pi)/180.0
    return theta_rad
