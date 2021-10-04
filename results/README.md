# Results 

## Control Methods 

This folder contains measured data files from all swing ups performed with different control methods on the real simple pendulum in the introduction video: **Differential Dynamic Programming (DDP), Direct Collocation, Gravity Compensation, Energy Shaping + Linear Quadratic Regulator,
Iterative Linear Quadratic Regulator (iLQR) and Soft Actor Critic (SAC)**
. 

The basic **Proportinal-Derivative (PD)** controller is used to demonstrate, that a non-dynamical swing up is not possible with torque limited to 1 Nm and therefore showcases how controllers for underactuated systems enhance the performance of a torque-limited system by exploiting the natural dynamics of the task.

The controller folders contain csv files where a trajectory is stored through position, velocity and torque data for each single time step. If the control method requires a precomputed trajectory the folder will contain a `dat_desired.csv` file along with a `data_measured.csv` file. Plots are used to compare desired and measured `position, velocity and torque data` in order to validate the performance of the individual controllers.   

## Parameter Identification

PD control is also used for dynamic Parameter identification. The required torques for model-based control can be measured using stiff position control and closely tracking the reference trajectory. The `/control_method_pd/system_identification` folder holds a text file containing the dynamically identified parameters of our test bed.

## Data Filter

Raw sensor data is often noisy. A convenient way to filter out undesirable frequencies is the **Fast Fourier Transform (FFT)**. First the power spectrum density (PSD) is computed, which helps to decompose the frequency bandwidth into its most dominant frequencies. With the help of the plots it is possible to decide which of the main frequencies shall be retained and what frequencies are less dense and can therefore be discarded as sensor noise. Plots and csv data files of the filter method are located in the `filter_method_fft` folder. 
