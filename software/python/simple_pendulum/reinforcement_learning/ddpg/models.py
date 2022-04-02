"""
Models
======
"""


import tensorflow as tf
from tensorflow.keras.layers import Input, Dense, Concatenate


def get_actor(state_shape, upper_bound=2.0, verbose=False):
    # Initialize weights between -3e-3 and 3-e3
    last_init = tf.random_uniform_initializer(minval=-0.003, maxval=0.003)

    inputs = Input(shape=(state_shape[0],))
    out = Dense(256, activation="relu")(inputs)
    out = Dense(256, activation="relu")(out)
    outputs = Dense(1, activation="tanh", kernel_initializer=last_init)(out)

    outputs = outputs * upper_bound
    model = tf.keras.Model(inputs, outputs)
    if verbose:
        print(model.summary())
    return model


def get_critic(state_shape, n_actions, verbose=False):
    # State as input
    state_input = Input(shape=(state_shape[0]))
    state_out = Dense(16, activation="relu")(state_input)
    state_out = Dense(32, activation="relu")(state_out)

    # Action as input
    action_input = Input(shape=(n_actions))
    action_out = Dense(32, activation="relu")(action_input)

    # Both are passed through seperate layer before concatenating
    concat = Concatenate()([state_out, action_out])

    out = Dense(256, activation="relu")(concat)
    out = Dense(256, activation="relu")(out)
    outputs = Dense(1)(out)

    # Outputs single value for give state-action
    model = tf.keras.Model([state_input, action_input], outputs)
    if verbose:
        print(model.summary())
    return model
