# Soft Actor Critic Control #

Type: Closed loop, learning based, model free

State/action space constraints: None

Optimal: Yes

Versatility: Swing-up and stabilization

## Theory # 

A controller class to use a model trained by the [sac trainer](https://github.com/dfki-ric-underactuated-lab/torque_limited_simple_pendulum/tree/master/software/python/simple_pendulum/reinforcement_learning/sac).

## Requirements # 
- Stable Baselines 3 (https://github.com/DLR-RM/stable-baselines3)
- Numpy
- PyYaml

## API # 

The class for the SAC Controller initialized by creating an instance as:

    controller = SacController(model_path=model_path,
                               torque_limit=2.0,
                               use_symmetry=True)
with the input:

- model_path: str or path
- torque_limit: torque_limit of the pendulum, the output of the model is scaled with this value
- use_symmetry: whether to use the left/right symmetry of the pendulum

The control output <img src="https://render.githubusercontent.com/render/math?math=\mathbf{u}(\mathbf{x})"> can be obtained with the API of the abstract controller class:

    SacController.get_control_output(mean_pos, mean_vel, meas_tau, meas_time)
        inputs:
            meas_pos: float, position of the pendulum
            meas_vel: float, velocity of the pendulum
            meas_tau: not used
            meas_time: not used
        returns:
            None, None, u

get_control_output returns None for the desired position and desired velocity (the sac controller is a pure torque controller). The returned torque u is the output of the trained model.

## Usage #
 An example of how to use this controller can be found in the [sim_sac.py script](https://github.com/dfki-ric-underactuated-lab/torque_limited_simple_pendulum/tree/master/software/python/simple_pendulum/reinforcement_learning/sac) in the examples folder.

A fully trained model is saved [here](https://github.com/dfki-ric-underactuated-lab/torque_limited_simple_pendulum/tree/master/data/models).

## Comments #






