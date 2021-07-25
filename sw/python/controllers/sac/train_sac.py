# Other imports
import os
import yaml
import shutil
from pathlib import Path
from stable_baselines import SAC
from stable_baselines.sac.policies import MlpPolicy
from stable_baselines.common.callbacks import EvalCallback, \
                                              StopTrainingOnRewardThreshold

# local imports
from model.pendulum_plant import PendulumPlant
from simulation.simulation import Simulator
from simulation.gym_environment import SimplePendulumEnv


# load training parameters
cwd = os.getcwd()
with open(os.path.join(cwd, 'training', 'parameters.yaml')) as fle:
    params = yaml.safe_load(fle)

# purge previous training files
potential_old_training_files = os.listdir('training')
if 'best_model' in potential_old_training_files:
    shutil.rmtree(os.path.join('training', 'best_model'))
if 'tb_logs' in potential_old_training_files:
    shutil.rmtree(os.path.join('training', 'tb_logs'))

# get the simulator
torque_limit = params['torque_limit']
mass = 0.546  # 0.546
length = 0.45  # 0.45
damping = 0.16
gravity = 9.81
coulomb_fric = 0
inertia = mass*length**2
pendulum = PendulumPlant(mass=mass,
                         length=length,
                         damping=damping,
                         gravity=gravity,
                         coulomb_fric=coulomb_fric,
                         inertia=inertia,
                         torque_limit=torque_limit)

simulator = Simulator(plant=pendulum)

# setup training gym environment
dt = float(params['dt'])
integrator = params['integrator']
env = SimplePendulumEnv(simulator=simulator,
                        max_steps=params['max_steps'],
                        reward_type=params['reward_type'],
                        dt=dt,
                        integrator=integrator)

# setup the agent
agent = SAC(MlpPolicy,
            env,
            verbose=1,
            tensorboard_log=os.path.join(cwd, 'training', 'tb_logs'),
            learning_rate=params['learning_rate'])
if params['warm_start'] is True:
    model_load_path = os.path.join(Path(__file__).parents[4],
                                   params['warm_start_path'])
    agent.load_parameters(load_path_or_dict=model_load_path)

# setup evaluation environment
eval_env = SimplePendulumEnv(simulator=simulator,
                             max_steps=params['max_steps'],
                             reward_type=params['reward_type'],
                             dt=dt,
                             integrator=integrator)

# define training callbacks
callback_on_best = StopTrainingOnRewardThreshold(reward_threshold=1000,
                                                 verbose=1)
eval_callback = EvalCallback(eval_env, callback_on_new_best=callback_on_best,
                             best_model_save_path=os.path.join(cwd, 'training', 'best_model'),
                             log_path=os.path.join(cwd, 'training', 'best_model'),
                             eval_freq=10000,
                             verbose=1,
                             n_eval_episodes=20)
# train
training_timesteps = int(1e6)
agent.learn(total_timesteps=training_timesteps,
            callback=eval_callback)

