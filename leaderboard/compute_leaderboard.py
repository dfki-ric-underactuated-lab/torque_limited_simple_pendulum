import os
import importlib
import pandas

from simple_pendulum.analysis.leaderboard import leaderboard_scores

def compute_leaderboard():
    data_paths = {}

    for f in os.listdir("."):
        if f[:4] == "con_":
            mod = importlib.import_module(f[:-3])
            if hasattr(mod, 'leaderboard_config'):
                if os.path.exists(mod.leaderboard_config["csv_path"]):
                    print(f"Found leaderboard_config and data for {mod.leaderboard_config['name']}")
                    data_paths[mod.leaderboard_config["name"]] = mod.leaderboard_config

    save_to = "data/leaderboard.csv"

    leaderboard_scores(data_paths,
                       save_to,
                       #weights={"swingup_time": 0.5, "max_tau": 0.1, "energy": 0.0, "integ_tau": 0.4, "tau_cost": 0.0, "tau_smoothness": 0.0},
                       weights={"swingup_time": 0.2, "max_tau": 0.1, "energy": 0.0, "integ_tau": 0.1, "tau_cost": 0.0, "tau_smoothness": 0.6},
                       normalize={"swingup_time": 10., "max_tau": 2.0, "energy": 1.0, "integ_tau": 20., "tau_cost": 40.0, "tau_smoothness": 4.0})

    print(pandas.read_csv(save_to).sort_values(by=["Real AI Score"], ascending=False).to_markdown(index=False))

if __name__ == "__main__":
    compute_leaderboard()
