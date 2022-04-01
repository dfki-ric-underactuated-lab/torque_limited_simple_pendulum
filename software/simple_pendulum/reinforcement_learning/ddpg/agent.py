"""
Agent
=====
"""


import numpy as np
import tensorflow as tf
import os


class Agent:
    def __init__(self,
                 state_shape,
                 n_actions,
                 action_limits,
                 discount,
                 actor_lr,
                 critic_lr,
                 actor_model,
                 critic_model,
                 target_actor_model,
                 target_critic_model,
                 tau=0.005):

        self.state_shape = state_shape
        self.n_actions = n_actions
        self.discount = discount

        self.actor = actor_model
        self.critic = critic_model

        self.target_actor = target_actor_model
        self.target_critic = target_critic_model

        self.update_target_weights(tau=1.0)

        self.actor_optimizer = tf.keras.optimizers.Adam(actor_lr)
        self.critic_optimizer = tf.keras.optimizers.Adam(critic_lr)

        self.tau = tau
        self.action_limits = action_limits

    def prep_state(self, state):
        return state

    def get_action(self, state, noise_object=None):

        tf_state = tf.expand_dims(tf.convert_to_tensor(state), 0)

        if noise_object is not None:
            noise = noise_object()
        else:
            noise = 0

        sampled_actions = tf.squeeze(self.actor(self.prep_state(tf_state)))
        sampled_actions = sampled_actions.numpy()
        sampled_actions += noise

        action = sampled_actions

        return np.squeeze(action)

    def scale_action(self, action, mini, maxi):
        a = action * (maxi - mini) + mini

        a = np.clip(a,
                    mini,
                    maxi)

        return a

    def train_on(self, batch):
        states, actions, next_states, rewards, done = batch

        with tf.GradientTape() as tape:
            target_actions = self.target_actor(self.prep_state(next_states),
                                               training=True)

            y = rewards + \
                self.discount*self.target_critic([self.prep_state(next_states),
                                                  target_actions],
                                                 training=True)
            critic_value = self.critic([np.atleast_2d(states),
                                        np.atleast_2d(actions)],
                                       training=True)
            critic_loss = tf.math.reduce_mean(tf.math.square(y - critic_value))

        critic_grad = tape.gradient(critic_loss,
                                    self.critic.trainable_variables)
        self.critic_optimizer.apply_gradients(
                        zip(critic_grad, self.critic.trainable_variables))

        with tf.GradientTape() as tape:
            act = self.actor(self.prep_state(states), training=True)
            critic_value = self.critic([self.prep_state(states), act],
                                       training=True)
            actor_loss = -tf.math.reduce_mean(critic_value)

        actor_grad = tape.gradient(actor_loss, self.actor.trainable_variables)
        self.actor_optimizer.apply_gradients(zip(actor_grad,
                                             self.actor.trainable_variables))

        return actor_loss, critic_loss

    def update_target_weights(self, tau=None):
        if tau is None:
            tau = self.tau

        for (a, b) in zip(self.target_actor.variables, self.actor.variables):
            a.assign(b * tau + a * (1 - tau))

        for (a, b) in zip(self.target_critic.variables, self.critic.variables):
            a.assign(b * tau + a * (1 - tau))

    def __prepare_batch(self, batch):
        return batch

    def save_model(self, path):
        if not os.path.exists(path):
            os.makedirs(path)
        self.target_actor.save(os.path.join(path, "actor"))
        self.target_critic.save(os.path.join(path, "critic"))

    def load_model(self, path):
        self.actor = tf.keras.models.load_model(os.path.join(path, "actor"),
                                                compile=True)
        self.critic = tf.keras.models.load_model(os.path.join(path, "critic"),
                                                 compile=True)

        self.update_target_weights(tau=1.0)
