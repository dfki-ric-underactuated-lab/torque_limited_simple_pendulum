import os
import importlib
import pandas
import argparse

from simple_pendulum.analysis.leaderboard import leaderboard_scores

from compute_leaderboard_data import compute_leaderboard_data


parser = argparse.ArgumentParser()
parser.add_argument(
    "--data-dir",
    dest="data_dir",
    help="Directory for saving data. Existing data will be kept.",
    default="data",
    required=False,
)
parser.add_argument(
    "--save_to",
    dest="save_to",
    help="Path for saving the leaderbaord csv file.",
    default="leaderboard.csv",
    required=False,
)
parser.add_argument(
    "--force-recompute",
    dest="recompute",
    help="Whether to force the recomputation of the leaderboard even without new data.",
    default=False,
    required=False,
    type=int,
)
parser.add_argument(
    "--link-base",
    dest="link",
    help="base-link for hosting data. Not needed for local execution",
    default="",
    required=False,
)

data_dir = parser.parse_args().data_dir
save_to = parser.parse_args().save_to
recompute_leaderboard = bool(parser.parse_args().recompute)
link_base = parser.parse_args().link

if not os.path.exists(save_to):
    recompute_leaderboard = True

data_paths = {}

for f in os.listdir("."):
    if f[:4] == "con_":
        mod = importlib.import_module(f[:-3])
        if hasattr(mod, "leaderboard_config"):
            if os.path.exists(
                os.path.join(data_dir, mod.leaderboard_config["csv_path"])
            ):
                print(
                    f"Found leaderboard_config and data for {mod.leaderboard_config['name']}"
                )
            else:
                print(f"Simulating controller {f}")
                compute_leaderboard_data(data_dir, f)
                recompute_leaderboard = True

            conf = mod.leaderboard_config
            conf["csv_path"] = os.path.join(
                data_dir, mod.leaderboard_config["csv_path"]
            )
            data_paths[mod.leaderboard_config["name"]] = conf

# save_to = f"{data_dir}/leaderboard.csv"

if recompute_leaderboard:
    leaderboard_scores(
        data_paths,
        save_to,
        # weights={"swingup_time": 0.5, "max_tau": 0.1, "energy": 0.0, "integ_tau": 0.4, "tau_cost": 0.0, "tau_smoothness": 0.0},
        weights={
            "swingup_time": 0.2,
            "max_tau": 0.1,
            "energy": 0.1,
            "integ_tau": 0.1,
            "tau_cost": 0.1,
            "tau_smoothness": 0.2,
            "velocity_cost": 0.2,
        },
        normalize={
            "swingup_time": 10.0,
            "max_tau": 2.0,
            "energy": 100.0,
            "integ_tau": 20.0,
            "tau_cost": 40.0,
            "tau_smoothness": 4.0,
            "velocity_cost": 100.0,
        },
        link_base=link_base,
        simulation=True,
    )

print(
    pandas.read_csv(save_to)
    .sort_values(by=["RealAI Score"], ascending=False)
    .to_markdown(index=False)
)
