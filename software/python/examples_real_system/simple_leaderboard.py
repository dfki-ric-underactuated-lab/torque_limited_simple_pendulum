import os
import numpy as np
import pandas as pd

def write_to_leaderboard(leaderboard_file, time):
    print(f"Your swingup time was {time}s.")

    inp = input("Do you want to add your score to the leaderboard? [Y/n]")

    if inp in ["", "Y", "y", "Yes", "yes", "Ja", "J", "ja", "j"]:
        name = input("Name: ")

        if not os.path.exists(leaderboard_file):
            d = {"Name": [name], "Time": [time]}
            lb_data = pd.DataFrame(data=d, index=[0])
        else:
            lb_data = pd.read_csv(leaderboard_file, index_col=0)
            new_data_point = pd.Series({"Name": name, "Time": time})

            lb_data = pd.concat([lb_data, new_data_point.to_frame().T], ignore_index=True)
        lb_data.sort_values(by=["Time"], ascending=True).to_csv(
            leaderboard_file, header=["Name", "Time"]
        )

if __name__ == "__main__":
    leaderboard_file = "data/simple_leaderboard.csv"
    time = 5 + np.random.rand()
    write_to_leaderboard(leaderboard_file, time)
