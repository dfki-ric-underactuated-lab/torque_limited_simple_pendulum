import matplotlib.pyplot as plt
import yaml


def plot_benchmarks(file_path, save_to):
    with open(file_path, "r") as f:
        scores = yaml.safe_load(f)

    labels = ["Consistency", "Robustness", "Sensitivity", "Min torque Score"]
    values = []
    values.append(scores["consistency"] / scores["iterations"])
    values.append(scores["robustness"] / scores["iterations"])
    values.append(scores["sensitivity"] / scores["iterations"])
    values.append((10.0 - scores["min_successful_torque"]) / 10.0)

    fig = plt.figure(figsize=(8, 6))
    plt.bar(labels, values, color=["blue", "red", "green", "orange"])
    plt.ylabel("Scores")
    plt.xlabel("Criteria")
    plt.savefig(save_to)
    plt.close()
