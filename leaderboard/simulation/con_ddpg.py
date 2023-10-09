import os

os.environ["TF_CPP_MIN_LOG_LEVEL"] = "3"
import tensorflow as tf

tf.get_logger().setLevel("ERROR")

from simple_pendulum.controllers.ddpg.ddpg_controller import ddpg_controller
from sim_parameters import torque_limit

name = "ddpg"
leaderboard_config = {
    "csv_path": "data/" + name + "/sim_swingup.csv",
    "name": name,
    "simple_name": "DDPG",
    "short_description": "RL Policy learned with Deep Deterministic Policy Gradient.",
    "readme_path": f"readmes/{name}.md",
    "username": "fwiebe",
}

torque_limit = 1.0
model_path = "../../data/models/ddpg_model/actor"
controller = ddpg_controller(
    model_path=model_path, torque_limit=torque_limit, state_representation=3
)
