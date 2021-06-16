# Other imports
import gym
import numpy as np


class SimplePendulumEnv(gym.Env):
    def __init__(self,
                 simulator,
                 max_steps=5000,
                 reward_type='continuous',
                 dt=1e-3,
                 integrator='runge_kutta'):
        self.simulator = simulator
        self.torque_limit = simulator.plant.torque_limit
        self.step_count = 0
        self.max_steps = max_steps
        self.reward_type = reward_type
        self.dt = dt
        self.integrator = integrator
        self.pos_target = np.pi
        self.action_space = gym.spaces.Box(-1, 1, shape=[1])  # using normalized action space, meaning we have to rescale the action later
        self.observation_space = gym.spaces.Box(np.array([-6 * np.pi, -20]), np.array([6 * np.pi, 20]))
        self.init_state = np.clip(np.random.randn(2), -np.pi, np.pi)
        self.simulator.set_state(0, self.init_state)

    def step(self, action):
        rescaled_action = self.torque_limit * action  # rescaling the action
        self.simulator.step(rescaled_action, self.dt, self.integrator)
        current_t, current_state = self.simulator.get_state()  # state is [position, velocity]
        observation = self.get_observation(current_state)
        reward = self.swingup_reward(observation)
        done = self.check_final_condition()
        info = {}
        self.step_count += 1

        return observation, reward, done, info

    def reset(self):
        self.simulator.reset_data_recorder()
        self.step_count = 0
        epsilon = np.random.rand()
        if epsilon < 0.99:
            pos_range = np.pi/10
            vel_range = np.pi/10
        else:
            pos_range = np.pi/10
            vel_range = np.pi/10
        self.init_state = np.array([np.random.rand()*2*pos_range - pos_range,
                                    np.random.rand()*2*vel_range - vel_range])
        self.simulator.set_state(0, self.init_state)
        current_t, current_state = self.simulator.get_state()
        observation = self.get_observation(current_state)

        return observation

    def render(self, mode='human'):
        pass

    def close(self):
        pass

    # some helper methods
    def get_observation(self, state):
        observation = np.array([obs for obs in state], dtype=np.float32)
        # wrap position
        observation[0] = (observation[0] + 6 * np.pi) % (np.pi * 6 * 2) - 6 * np.pi

        return observation

    def swingup_reward(self, state):
        reward = None
        pos = state[0]
        pos = np.abs((pos + np.pi) % (np.pi * 2) - np.pi)
        if self.reward_type == 'soft_binary_with_repellor':
            pos_diff = self.pos_target - pos
            reward = np.exp(-pos_diff ** 2 / (2 * 0.25 ** 2))
            pos_diff_repellor = pos - 0
            reward -= np.exp(-pos_diff_repellor ** 2 / (2 * 0.25 ** 2))
        else:
            raise NotImplemented(f'Sorry, the reward type {self.reward_type} is not implemented.')

        return reward

    def check_final_condition(self):
        done = False
        if self.step_count > self.max_steps:
            done = True

        return done

