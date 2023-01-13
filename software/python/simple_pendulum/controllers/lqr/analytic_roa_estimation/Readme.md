# Analytic Estimation of Region of Attraction of an LQR Controller for Torque Limited Simple Pendulum

This repository contains the Code used in the work on ["Analytic Estimation of Region of Attraction of an LQR Controller for Torque Limited Simple Pendulum"](https://dfki-ric-underactuated-lab.github.io/analytic_roa_lqr_pendulum/) of Gross et al. (to be published in Proceedings of IEEE CDC 2022).

### Installation

To run and experiment with the code, first you need to follow the [installation guide](https://github.com/dfki-ric-underactuated-lab/torque_limited_simple_pendulum/blob/master/docs/reference/source/chapters/00.installation_guide.rst).

For the plots, the python package SciencePlots is needed. It can be installed via:
```
pip install scienceplots
```

### Usage

In the folder [notebooks](notebooks) you can find a jupyter notebook called [sim_data_generate](notebooks/sim_data_generate.ipynb) that runs the simulation used to collect numerical data. [analysis_and_plotting](notebooks/analysis_and_plotting.ipynb) does the region of attraction analysis on the data and generates plots, comparing the analytic approach to the sampling based approach. [analytic_roa](analytic_roa.py) contains the code, which implements the oracle-style function using the analytic approach, [najafi_oracle](najafi_oracle.py) implements the sampling-based version. For a more detailed description of the theory, refer to the technical paper.

### Citation
L. Gross, L. Maywald, S.Kumar, F. Kirchner, C. Lüth, "Analytic Estimation of Region of Attraction of an LQR Controller for Torque Limited Simple Pendulum," 2022 61st IEEE Conference on Decision and Control (CDC), 2022.

```
@INPROCEEDINGS{9992856,
  author={Gross, Lukas and Maywald, Lasse and Kumar, Shivesh and Kirchner, Frank and Lüth, Christoph},
  booktitle={2022 IEEE 61st Conference on Decision and Control (CDC)}, 
  title={Analytic Estimation of Region of Attraction of an LQR Controller for Torque Limited Simple Pendulum}, 
  year={2022},
  volume={},
  number={},
  pages={2695-2701},
  doi={10.1109/CDC51059.2022.9992856}}
```
