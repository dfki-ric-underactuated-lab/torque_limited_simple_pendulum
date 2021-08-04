# Other imports
import gym
import numpy as np


class SimplePendulumEnv(gym.Env):
    """
    An environment for reinforcement learning
    """
    def __init__(self,
                 simulator,
                 max_steps=5000,
                 reward_type='continuous',
                 state_target_epsilon=[1e-2, 1e-2],
                 dt=1e-3,
                 integrator='runge_kutta'):
        """
        An environment for reinforcement learning.

        Parameters
        ----------
        simulator : simulator object
        max_steps : int, default=5000
                maximum steps the agent can take before the episode
                is terminated
        reward_type : string, default='continuous'
                the reward type selects the reward function which is used
                options are: 'continuous', 'discrete', 'soft_binary',
                             'soft_binary_with_repellor'
        state_target_epsilon: array-like, default=[1e-2, 1e-2]
                target epsilon for discrete reward type
        dt : float, default=1e-3
            timestep for the simulation
        integrator : string, default='runge_kutta'
            the integrator which is used by the simulator
            options : 'euler', 'runge_kutta'
        """
        self.simulator = simulator
        self.torque_limit = simulator.plant.torque_limit
        self.step_count = 0
        self.max_steps = max_steps
        self.reward_type = reward_type
        self.state_target_epsilon = state_target_epsilon
        self.dt = dt
        self.integrator = integrator
        self.pos_target = np.pi

        # using normalized action space,
        # meaning we have to rescale the action later
        self.action_space = gym.spaces.Box(-1, 1, shape=[1])
        self.observation_space = gym.spaces.Box(np.array([-6 * np.pi, -20]),
                                                np.array([6 * np.pi, 20]))
        self.init_state = np.clip(np.random.randn(2), -np.pi, np.pi)
        self.simulator.set_state(0, self.init_state)

    def step(self, action):
        """
        Take a step in the environment.

        Parameters
        ----------
        action : float
            the normalized (i.e. in the range [-1, 1])
            torque that is applied to the pendulum

        Returns
        -------
        observation : array-like
            the observation from the environment after the step
        reward : float
            the reward received on this step
        done : bool
            whether the episode has terminated
        info : dictionary
            may contain additional information
            (empty at the moment)
        """
        rescaled_action = self.torque_limit * action  # rescaling the action
        self.simulator.step(rescaled_action, self.dt, self.integrator)
        current_t, current_state = self.simulator.get_state()
        # current_state is [position, velocity]
        observation = self.get_observation(current_state)
        reward = self.swingup_reward(observation)
        done = self.check_final_condition()
        info = {}
        self.step_count += 1

        return observation, reward, done, info

    def reset(self):
        """
        Reset the environment. The pendulum is initialized with a random state
        in the vicinity of the stable fixpoint
        (position and velocity are in the range[-0.31, 0.31])

        Parameters
        ----------

        Returns
        -------
        observation : array-like
            the state the pendulum has been initilized to
        """
        self.simulator.reset_data_recorder()
        self.step_count = 0
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
        """
        Transform the state from the simulator an observation by
        wrapping the position to the observation space

        Parameters
        ----------
        state : array-like
            state as output by the simulator

        Returns
        -------
        observation : array-like
            observation in environment format
        """
        observation = np.array([obs for obs in state], dtype=np.float32)
        # wrap position
        observation[0] = (observation[0] + 6*np.pi) % (np.pi*6*2) - 6*np.pi

        return observation

    def swingup_reward(self, observation):
        """
        Calculate the reward for the pendulum for swinging up to the instable
        fixpoint. The reward function is selected based on the reward type
        defined during the object inizialization.

        Parameters
        ----------
        state : arraylike
            the observation that has been received from the environment

        Returns
        -------
        reward : float
            the reward for swinging up

        Raises
        ------
        NotImplementedError
            when the requested reward_type is not implemented

        """
        reward = None
        pos = observation[0]
        pos = np.abs((pos + np.pi) % (np.pi * 2) - np.pi)
        if self.reward_type == 'continuous':
            reward = - np.linalg.norm(pos - self.target[0])
        elif self.reward_type == 'discrete':
            reward = np.float(np.linalg.norm(pos - self.target[0]) <
                              self.state_target_epsilon[0])
        elif self.reward_type == 'soft_binary':
            pos_diff = self.target[0] - pos
            reward = np.exp(-pos_diff**2/(2*0.25**2))
        elif self.reward_type == 'soft_binary_with_repellor':
            pos_diff = self.pos_target - pos
            reward = np.exp(-pos_diff ** 2 / (2 * 0.25 ** 2))
            pos_diff_repellor = pos - 0
            reward -= np.exp(-pos_diff_repellor ** 2 / (2 * 0.25 ** 2))
        else:
            raise NotImplementedError(f'Sorry, the reward type {self.reward_type} is not implemented.')

        return reward

    def check_final_condition(self):
        """
        Checks whether a terminating condition has been met.
        The only terminating condition for the pendulum is if the maximum
        number of steps has been reached.

        Returns
        -------
        done : bool
            whether a terminating condition has been met
        """
        done = False
        if self.step_count > self.max_steps:
            done = True

        return done
