import numpy as np

from model.pendulum_plant import PendulumPlant

MASSES = [0.01, 0.1, 1.0, 10.0, 100.0]
LENGTHS = [0.01, 0.1, 1.0, 10.0, 100.0]
DAMPINGS = [0.0, 0.01, 0.1, 1.0]
GRAVITY = [0.0, 9.81]
CFRICS = [0.0, 0.01, 0.1, 1.0]
TLIMITS = [1.0, 2.0, 3.0, 5.0, np.inf]

iterations = 100
epsilon = 1e-4

max_angle = 5.0
max_vel = 5.0
max_tau = 5.0

# kinematics test
kinematics_test_passed = True
for length in LENGTHS:
    pendulum = PendulumPlant(length=length)
    for _ in range(iterations):
        angle = (np.random.rand() - 0.5) * 2*max_angle
        ee_pos = pendulum.forward_kinematics(angle)[0]
        ret_angle = pendulum.inverse_kinematics(ee_pos)
        if np.abs((angle % (2*np.pi)) - (ret_angle % (2*np.pi))) > epsilon:
            kinematics_test_passed = False
            print("Kinematics test failed. \
                  length {}, \
                  angle {}, \
                  ee_pos {}, \
                  returned angle {}".format(length,
                                            angle,
                                            ee_pos,
                                            ret_angle))

print("Kinematics test passed: {}".format(kinematics_test_passed))

# dynamics test
dynamics_test_passed = True
for mass in MASSES:
    for length in LENGTHS:
        for damping in DAMPINGS:
            for gravity in GRAVITY:
                for coulomb_fric in CFRICS:
                    for torque_limit in TLIMITS:
                        inertia = mass*length*length
                        pendulum = PendulumPlant(mass=mass,
                                                 length=length,
                                                 damping=damping,
                                                 gravity=gravity,
                                                 coulomb_fric=coulomb_fric,
                                                 inertia=inertia,
                                                 torque_limit=torque_limit)
                        angle = (np.random.rand() - 0.5) * 2*max_angle
                        vel = (np.random.rand() - 0.5) * 2*max_vel
                        tau = (np.random.rand() - 0.5) * 2*max_tau
                        state = [angle, vel]

                        accn = pendulum.forward_dynamics(state, tau)
                        ret_tau = pendulum.inverse_dynamics(state, accn)
                        clipped_tau = np.clip(tau, -torque_limit, torque_limit)
                        if np.abs(clipped_tau - ret_tau) > epsilon:
                            dynamics_test_passed = False
                            print("Dynamics test failed.\
                                  mass {},\
                                  length {}, \
                                  damping {}, \
                                  gravity {}, \
                                  coulomb friction {}, \
                                  state {}, \
                                  torque {}, \
                                  returned torque {}".format(mass,
                                                             length,
                                                             damping,
                                                             gravity,
                                                             coulomb_fric,
                                                             state,
                                                             tau,
                                                             ret_tau))

print("Dynamics test passed: {}".format(dynamics_test_passed))
