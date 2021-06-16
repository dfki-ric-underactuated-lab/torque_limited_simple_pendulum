def profiler(n, dt, des_time, meas_time, start, end, meas_dt):
    # validate avg dt of the control loop with (start time - end time) / numSteps
    meas_avg_dt = (end - start)/n

    print("Performance Profiler:")
    print(                                                                  # Compare desired versus measured control frequency
        f"{'Desired frequency:  ':>40}{(1/dt):<20}{' Hz':<20}",
        f"\n{'Measured avg frequency:  ':>40}{(1/meas_avg_dt):<20}{' Hz':<20}",
        f"\n{'Measured sample frequency:  ':>40}{(1/meas_dt):<20}{' Hz':<20}",
        f"\n{'Difference in Meas-Des frequency:  ':>40}{(abs(1/(dt-meas_avg_dt))):<20}{' Hz':<20}",
    )
    print()
    print(
        f"{'Desired dt:  ':>40}{dt:<21}{' s':<20}",
        f"\n{'Measured avg dt:  ':>40}{meas_avg_dt:<21}{' s':<20}",
        f"\n{'Measured sample dt:  ':>40}{meas_dt:<21}{' s':<20}",
        f"\n{'Time error Meas-Des dt:  ':>40}{abs((dt-meas_avg_dt)*1000):<21}{' milliseconds':<20}",
    )
    print()
    print("Time total desired:", des_time[n - 1], "s")
    print("Time total measured:", meas_time[n - 1], "s")
    #print("meas_time:", len(meas_time))
    #print("des_time:", len(des_time))
    #print("meas_pos:", len(meas_pos))
    #print("des_pos:", len(des_pos))
    print()