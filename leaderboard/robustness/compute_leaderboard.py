import os
import importlib
import argparse
import pandas
import numpy as np
import yaml

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
    "--force-recompute",
    dest="recompute",
    help="Whether to force the recomputation of the leaderboard even without new data.",
    default=False,
    required=False,
    type=bool,
)

parser.add_argument(
    "--link-base",
    dest="link",
    help="base-link for hosting data. Not needed for local execution",
    default="",
    required=False,
)

data_dir = parser.parse_args().data_dir
recompute_leaderboard = parser.parse_args().recompute
link_base = parser.parse_args().link

if not os.path.exists(data_dir):
    os.makedirs(data_dir)

existing_list = os.listdir(data_dir)
for con in existing_list:
    if not os.path.exists(os.path.join(data_dir, con, "benchmark.yml")):
        existing_list.remove(con)

for file in os.listdir("."):
    if file[:4] == "con_":
        if file[4:-3] in existing_list:
            print(f"Robustness benchmark data for {file} found.")
        else:
            print(f"Creating benchmarks for new controller {file}")
            compute_leaderboard_data(data_dir, file)
            recompute_leaderboard = True

if recompute_leaderboard:
    src_dir = "."
    save_to = os.path.join(data_dir, "leaderboard.csv")

    leaderboard_data = []

    for f in os.listdir(src_dir):
        if f[:4] == "con_":
            mod = importlib.import_module(f[:-3])
            if hasattr(mod, "leaderboard_config"):
                conf = mod.leaderboard_config
                if os.path.exists(
                    os.path.join(data_dir, conf["name"], "benchmark.yml")
                ):
                    print(
                        f"Found leaderboard_config and data for {mod.leaderboard_config['name']}"
                    )

                    with open(
                        os.path.join(data_dir, conf["name"], "benchmark.yml"),
                        "r",
                    ) as f:
                        scores = yaml.safe_load(f)

                    final_score = (
                        0.25 * scores["consistency"] / scores["iterations"]
                        + 0.25 * scores["robustness"] / scores["iterations"]
                        + 0.25 * scores["sensitivity"] / scores["iterations"]
                        + 0.25 * (10.0 - scores["min_successful_torque"]) / 10.0
                    )

                    if link_base != "":
                        if "simple_name" in conf.keys():
                            name_with_link = f"[{conf['simple_name']}]({link_base}{conf['name']}/README.md)"
                        else:
                            name_with_link = (
                                f"[{conf['name']}]({link_base}{conf['name']}/README.md)"
                            )
                    else:
                        if "simple_name" in conf.keys():
                            name_with_link = conf["simple_name"]
                        else:
                            name_with_link = conf["name"]

                    append_data = [
                        name_with_link,
                        conf["short_description"],
                        "{:.1f}".format(
                            100 * scores["consistency"] / scores["iterations"]
                        ),
                        "{:.1f}".format(
                            100 * scores["robustness"] / scores["iterations"]
                        ),
                        "{:.1f}".format(
                            100 * scores["sensitivity"] / scores["iterations"]
                        ),
                        "{:.1f}".format(scores["min_successful_torque"]),
                        "{:.3f}".format(final_score),
                        conf["username"],
                    ]
                    if link_base != "":
                        append_data.append(
                            "[Data and Plots](" + link_base + conf["name"] + ")"
                        )

                    leaderboard_data.append(append_data)

    header = "Controller,Short Controller Description,Consistency [%],Robustness [%],Sensitivity [%],Min torque [Nm],Overall Robustness Score,Username"

    if link_base != "":
        header += ",Data"

    np.savetxt(
        save_to,
        leaderboard_data,
        header=header,
        delimiter=",",
        fmt="%s",
        comments="",
    )
    df = pandas.read_csv(save_to)
    df = df.drop(df.columns[1], axis=1)
    print(
        df.sort_values(by=["Overall Robustness Score"], ascending=False).to_markdown(
            index=False
        )
    )
