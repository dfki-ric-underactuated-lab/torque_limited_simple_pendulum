from simple_pendulum.reinforcement_learning.sac.sac import sac_trainer


# pendulum parameters
torque_limit = 1.0
mass = 0.6755
length = 0.5
damping = 0.1
gravity = 9.81
coulomb_fric = 1.3
inertia = mass*length**2

# environment parameters
dt = 0.01
integrator = "runge_kutta"
max_steps = 1000
reward_type = "soft_binary_with_repellor"

# training parameters
learning_rate = 0.0003
training_timesteps = 1e6
reward_threshold = 1000
eval_frequency = 10000
n_eval_episodes = 20


trainer = sac_trainer(log_dir="temp/sac_training")

trainer.init_pendulum(mass=mass,
                      length=length,
                      inertia=inertia,
                      damping=damping,
                      coulomb_friction=coulomb_fric,
                      gravity=gravity,
                      torque_limit=torque_limit)

trainer.init_environment(dt=dt,
                         integrator=integrator,
                         max_steps=max_steps,
                         reward_type=reward_type)

trainer.init_agent(learning_rate=learning_rate,
                   warm_start=False,
                   warm_start_path="",
                   verbose=1)
trainer.train(training_timesteps=training_timesteps,
              reward_threshold=reward_threshold,
              eval_frequency=eval_frequency,
              n_eval_episodes=n_eval_episodes,
              verbose=1)
