# Deep Deterministic Policy Gradient Control #

Type: Closed loop, learning based, model free

State/action space constraints: None

Optimal: Yes

Versatility: Swing-up and stabilization

## Theory #

A controller class to use a model trained by the [ddpg trainer](https://github.com/dfki-ric-underactuated-lab/torque_limited_simple_pendulum/tree/master/software/python/simple_pendulum/reinforcement_learning/ddpg).

## Requirements #
- Tensorflow 2.x

## API #

The class for the DDPG Controller initialized by creating an instance as:

    controller = ddpg_controller(model_path=model_path,
                                 torque_limit=2.0,
                                 state_representation=3)
with the input:

- model_path: str or path, path to the actor model
- torque_limit: torque_limit of the pendulum, the output of the model is scaled with this value
- state_representation: =2 or =3, How the state is represented in the ddpg model

The control output <img src="https://render.githubusercontent.com/render/math?math=\mathbf{u}(\mathbf{x})"> can be obtained with the API of the abstract controller class:

    ddpg_controller.get_control_output(mean_pos, mean_vel, meas_tau, meas_time)
        inputs:
            meas_pos: float, position of the pendulum
            meas_vel: float, velocity of the pendulum
            meas_tau: not used
            meas_time: not used
        returns:
            None, None, u

get_control_output returns None for the desired position and desired velocity (the ddpg controller is a pure torque controller). The returned torque u is the output of the trained model.

## Usage #
 An example of how to use this controller can be found in the [sim_ddpg.py script](https://github.com/dfki-ric-underactuated-lab/torque_limited_simple_pendulum/blob/master/software/python/examples/sim_ddpg.py) in the examples folder.

A fully trained model is saved [here](https://github.com/dfki-ric-underactuated-lab/torque_limited_simple_pendulum/tree/master/data/models/ddpg_model/actor).

## Comments #






