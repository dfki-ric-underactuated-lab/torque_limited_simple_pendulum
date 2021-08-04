import numpy as np
import pydrake.symbolic as sym


def pendulum_continuous_dynamics(x, u, m=0.5, l=0.5,
                                 b=0.15, cf=0.0, g=9.81, inertia=0.125):
    md = sym if x.dtype == object else np  # Check type for autodiff
    pos = x[0]
    vel = x[1]
    coulomb_fric = 0.0
    if md == sym:
        accn = (u[0] - m * g * l * md.sin(pos) - b * vel -
                coulomb_fric*md.atan(1e8*vel)*2/np.pi) / inertia
    if md == np:
        accn = (u[0] - m * g * l * md.sin(pos) - b * vel -
                coulomb_fric*md.arctan(1e8*vel)*2/np.pi) / inertia
    xd = np.array([vel, accn])
    return xd


def pendulum_discrete_dynamics_euler(x, u, dt, m=0.57288, l=0.5, b=0.15,
                                     cf=0.0, g=9.81, inertia=0.125):
    x_d = pendulum_continuous_dynamics(x, u, m=m, l=l, b=b, cf=cf,
                                       g=g, inertia=inertia)
    x_next = x + x_d*dt
    return x_next


def pendulum_discrete_dynamics_rungekutta(x, u, dt, m=0.5, l=0.5,
                                          b=0.15, cf=0.0, g=9.81, inertia=0.125):
    k1 = pendulum_continuous_dynamics(x, u, m=m, l=l, b=b, cf=cf,
                                      g=g, inertia=inertia)
    k2 = pendulum_continuous_dynamics(x+0.5*dt*k1, u, m=m, l=l, b=b,
                                      cf=cf, g=g, inertia=inertia)
    k3 = pendulum_continuous_dynamics(x+0.5*dt*k2, u, m=m, l=l, b=b,
                                      cf=cf, g=g, inertia=inertia)
    k4 = pendulum_continuous_dynamics(x+dt*k3, u, m=m, l=l, b=b,
                                      cf=cf, g=g, inertia=inertia)
    x_d = (k1 + 2 * (k2 + k3) + k4) / 6.0
    x_next = x + x_d*dt
    return x_next


def pendulum_swingup_stage_cost(x, u, goal=[np.pi, 0], Cu=10.0, Cp=0.01,
                                Cv=0.01, Cen=0.0, m=0.5, l=0.5, b=0.15,
                                cf=0.0, g=9.81):
    md = sym if x.dtype == object else np  # Check type for autodiff
    eps = 1e-6
    c_pos = (x[0] - goal[0] + eps)**2.0
    c_vel = (x[1] - goal[1] + eps)**2.0
    c_control = u[0]**2
    en_g = 0.5*m*(l*goal[1])**2.0 + m*g*l*(1.0-md.cos(goal[0]))
    en = 0.5*m*(l*x[1])**2.0 + m*g*l*(1.0-md.cos(x[0]))
    c_en = (en-en_g+eps)**2.0
    return Cu*c_control + Cp*c_pos + Cv*c_vel + Cen*c_en


def pendulum_swingup_final_cost(x, goal=[np.pi, 0], Cp=1000.0, Cv=10.0,
                                Cen=0.0, m=0.5, l=0.5, b=0.15,
                                cf=0.0, g=9.81):
    md = sym if x.dtype == object else np  # Check type for autodiff
    eps = 1e-6
    c_pos = (x[0] - goal[0] + eps)**2.0
    c_vel = (x[1] - goal[1] + eps)**2.0
    en_g = 0.5*m*(l*goal[1])**2.0 + m*g*l*(1.0-md.cos(goal[0]))
    en = 0.5*m*(l*x[1])**2.0 + m*g*l*(1.0-md.cos(x[0]))
    c_en = (en-en_g+eps)**2.0
    return Cp*c_pos + Cv*c_vel + Cen*c_en


def pendulum3_discrete_dynamics_euler(x, u, dt, m=0.5, l=0.5, b=0.15,
                                      cf=0.0, g=9.81, inertia=0.125):
    # pendulum state x = [cos(theta), sin(theta), thetadot]
    md = sym if x.dtype == object else np  # Check type for autodiff
    if md == np:
        x2 = np.array([np.arctan2(x[1], x[0]), x[2]])
    if md == sym:
        x2 = np.array([sym.atan2(x[1], x[0]), x[2]])
    x_next = pendulum_discrete_dynamics_euler(x2, u, dt, m=m, l=l,
                                              b=b, cf=cf, g=g, inertia=inertia)
    x3 = np.array([md.cos(x_next[0]), md.sin(x_next[0]), x_next[1]])
    return x3


def pendulum3_discrete_dynamics_rungekutta(x, u, dt, m=0.5, l=0.5, b=0.15,
                                           cf=0.0, g=9.81, inertia=0.125):
    # pendulum state x = [cos(theta), sin(theta), thetadot]
    md = sym if x.dtype == object else np  # Check type for autodiff
    if md == np:
        x2 = np.array([np.arctan2(x[1], x[0]), x[2]])
    if md == sym:
        x2 = np.array([sym.atan2(x[1], x[0]), x[2]])
    x_next = pendulum_discrete_dynamics_rungekutta(x2, u, dt, m=m,
                                                   l=l, b=b, cf=cf, g=g,
                                                   inertia=inertia)
    x3 = np.array([md.cos(x_next[0]), md.sin(x_next[0]), x_next[1]])
    return x3


def pendulum3_swingup_stage_cost(x, u, goal=[-1, 0, 0], Cu=10.0, Cp=0.01,
                                 Cv=0.01, Cen=0.0, m=0.5, l=0.5, b=0.15,
                                 cf=0.0, g=9.81):
    # pendulum state x = [cos(theta), sin(theta), thetadot]
    md = sym if x.dtype == object else np  # Check type for autodiff
    if md == np:
        x2 = np.array([np.arctan2(x[1], x[0]), x[2]])
    if md == sym:
        x2 = np.array([sym.atan2(x[1], x[0]), x[2]])
    eps = 1e-6
    c_pos1 = (x[0] - goal[0] + eps)**2.0
    c_pos2 = (x[1] - goal[1] + eps)**2.0
    c_vel = (x[2] - goal[2] + eps)**2.0
    c_control = u[0]**2
    en_g = 0.5*m*(l*goal[2])**2.0 + m*g*l*(1.0-goal[0])
    en = 0.5*m*(l*x[2])**2.0 + m*g*l*(1.0-x2[0])
    c_en = (en-en_g+eps)**2.0
    return Cu*c_control + Cp*(c_pos1+c_pos2) + Cv*c_vel + Cen*c_en


def pendulum3_swingup_final_cost(x, goal=[-1, 0, 0], Cp=1000.0, Cv=10.0,
                                 Cen=0.0, m=0.57288, l=0.5, b=0.15,
                                 cf=0.0, g=9.81):
    # pendulum state x = [cos(theta), sin(theta), thetadot]
    md = sym if x.dtype == object else np  # Check type for autodiff
    if md == np:
        x2 = np.array([np.arctan2(x[1], x[0]), x[2]])
    if md == sym:
        x2 = np.array([sym.atan2(x[1], x[0]), x[2]])
    eps = 1e-6
    c_pos1 = (x[0] - goal[0] + eps)**2.0
    c_pos2 = (x[1] - goal[1] + eps)**2.0
    c_vel = (x[2] - goal[2] + eps)**2.0
    en_g = 0.5*m*(l*goal[2])**2.0 + m*g*l*(1.0-goal[0])
    en = 0.5*m*(l*x[2])**2.0 + m*g*l*(1.0-x2[0])
    c_en = (en-en_g+eps)**2.0
    return Cp*(c_pos1+c_pos2) + Cv*c_vel + Cen*c_en
