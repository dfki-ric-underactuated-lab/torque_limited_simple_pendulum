import numpy as np

def discretize_pos_vel(p, v, pos_res, vel_res):
    p = (p + np.pi) % (2*np.pi) - np.pi
    p = np.around(p / pos_res)*pos_res
    v = np.around(v / vel_res)*vel_res
    p = np.around(p, 3)
    v = np.around(v, 2)
    return p, v

def find_min_max_velocity_for_swingup(mass, length, gravity):
    E_pot = mass*gravity*2*length
    v = np.sqrt(np.abs(E_pot / mass / 0.5))
    tv = v / length
    return tv


def pos_vel_to_index(pos, vel, pos_res, pos_min, pos_max, vel_res, vel_min, vel_max):
    n_pos = (pos_max - pos_min) / pos_res
    n_vel = (vel_max - vel_min) / vel_res

    pos_id = np.around((pos - pos_min) / pos_res)
    vel_id = np.around((vel - vel_min) / vel_res)

    index = n_pos * vel_id + pos_id
    return int(index)

def index_to_pos_vel(index, pos_res, pos_min, pos_max, vel_res, vel_min, vel_max):
    n_pos = np.around((pos_max - pos_min) / pos_res)
    n_vel = np.around((vel_max - vel_min) / vel_res)

    pos_id = index % n_pos
    vel_id = (index - pos_id) / n_pos

    pos = pos_id * pos_res + pos_min
    vel = vel_id * vel_res + vel_min

    return pos, vel

