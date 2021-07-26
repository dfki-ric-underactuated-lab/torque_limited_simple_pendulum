# How to Contribute

If you want to contribute to the project, i.e. by designing new controllers, you are very welcome to do so. 

If you implement a controller please consider the following guidelines:

1. Create a folder with a unique and descriptive name for your controller in [sw/python/controllers](sw/python/controllers).
2. Make sure your controller inherits from the [AbstractController](sw/python/controllers/abstract_controller.py) class.
3. Create a README.md file in your controller directory in which you explain how the controller works and how it can be used.
4. Create a requirements.txt file where you list required packages that are not listed in the main [requirements](sw/python/requirements.txt) file of the project.


If you implement an offline trajectory optimization or reinforcement learning algorithm follow these guidelines:

1. Create a folder with a unique and descriptive name for your controller in [sw/python/trajectory_optimization](sw/python/trajectory_optimization).
2. Create a README.md file in your controller directory in which you explain how the method works and how it can be used.
3. Create a requirements.txt file where you list required packages that are not listed in the main [requirements](sw/python/requirements.txt) file of the project.
4. Either

    (a) write a controller class which makes it possible to execute your policy in simulation and on the real system. Make sure it  inherits from the [AbstractController](sw/python/controllers/abstract_controller.py) class.

    (b) export your trajectory to a csv file and save it at [data/trajectories](data/trajectories). The columns of the csv file should be: time, position, velocity, control_inputs. The first line is reserved for a header. In this format the csv file can be simulated or applied to the real system with the [open loop controller class](sw/python/controllers/open_loop/README.md)


If you want to contribute software in another programming language besides python, please create a new directory in the [software folder](sw) and try to maintain a similar structure as in the python directory.
