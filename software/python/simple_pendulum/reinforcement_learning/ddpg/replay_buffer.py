"""
Replay Buffer
=============
"""


import numpy as np
import tensorflow as tf


class ReplayBuffer:
    """
    Replay buffer class to store experiences for a
    reinforcement learning agent.
    """
    def __init__(self, max_size, num_states, num_actions):
        """
        Replay buffer class to store experiences for a
        reinforcement learning agent.

        Parameters
        ----------
        max_size: int
            maximum number of experiences to store in the repleay buffer.
            When adding experiences beyond this limit, the first entry
            is deleted.
        num_state: int
            the dimension of the state space
        num_actions: int
            the dimension of the action space
        """
        self.buffer_capacity = max_size
        self.num_states = num_states
        self.num_actions = num_actions

        self.clear()

    def append(self, obs_tuple):
        """
        Add an experience to the replay buffer.
        When adding experiences beyond the max_size limit,
        the first entry is deleted.
        An observation consists of (state, action, next_state, reward, done)

        Parameters
        ----------
        obs_tuple: array-like
            an observation (s,a,s',r,d) to store in the buffer
        """
        index = self.size % self.buffer_capacity

        self.state_buffer[index] = obs_tuple[0]
        self.action_buffer[index] = obs_tuple[1]
        self.next_state_buffer[index] = obs_tuple[2]
        self.reward_buffer[index] = obs_tuple[3]
        self.done_buffer[index] = obs_tuple[4]

        self.size += 1

    def sample_batch(self, batch_size):
        """
        Sample a batch from the replay buffer.

        Parameters
        ----------
        batch_size: int
            number of samples in the returned batch

        Returns
        -------
        tuple
            (s_batch,a_batch,s'_batch,r_batch,d_batch)
            a tuple of batches of state, action, reward, next_state, done
        """
        record_range = min(self.size, self.buffer_capacity)
        batch_indices = np.random.choice(record_range, batch_size)

        # Convert to tensors
        state_batch = tf.convert_to_tensor(self.state_buffer[batch_indices])
        action_batch = tf.convert_to_tensor(self.action_buffer[batch_indices])
        reward_batch = tf.convert_to_tensor(self.reward_buffer[batch_indices])
        reward_batch = tf.cast(reward_batch, dtype=tf.float32)
        next_state_batch = tf.convert_to_tensor(
                    self.next_state_buffer[batch_indices])
        done_batch = self.done_buffer[batch_indices]
        return (state_batch, action_batch, next_state_batch,
                reward_batch, done_batch)

    def clear(self):
        """
        Clear the Replay Buffer.
        """
        self.state_buffer = np.zeros((self.buffer_capacity, self.num_states))
        self.action_buffer = np.zeros((self.buffer_capacity, self.num_actions))
        self.next_state_buffer = np.zeros((self.buffer_capacity,
                                           self.num_states))
        self.reward_buffer = np.zeros((self.buffer_capacity, 1))
        self.done_buffer = np.zeros((self.buffer_capacity, 1))
        self.size = 0
