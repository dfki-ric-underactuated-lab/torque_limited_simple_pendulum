<div align="center">

#  Simple Pendulum
</div>

## Introduction #

The project is an open-source, low-cost, robust, and modular kit to get started with dynamic robots. The kit targets lowering the barrier for studying dynamic robots and legged locomotion in real systems. Controlling dynamic motions in real robots present unique challenges to the software and hardware which are often overlooked in conventional robotics courses. This project describes the topics which can be studied using the kit, lists its components, discusses best practices for implementation, presents results from experiments with the simulator and the real system.

LICENSE: All files contained in this repository, unless otherwise noted, are
available under an (...) License:


See the simple pendulum in action here:
https://youtu.be/_lbKIpiRWKI

## Documentation 

* [Initial Setup](docs/getting_started.md)
* [Dynamics](docs/getting_started.md)
* [PID-Controller](docs/getting_started.md)


## Directory Structure #

<table>
	<tr>
        <td><ul>
                <li>data/
                <ul>
                    <li>URDF</li>
                    <li>trajectories/</li>
                </ul>
                <li>docs/</li>
                <li>hw/
                <ul>
                    <li>CAD/</li>
                </ul>
                <li>results/</li>
                <li>sw/
                <ul>
                    <li>cpp/
                    <ul>
                        <li>controllers/</li>
                    </ul>
                    <li>python/
                    <ul>
                        <li>controllers/</li>
                        <li>model/</li>
                        <li>simulation/</li>
                        <li>utils/</li>
                </ul>
                </ul>
            </ul></td>    
		<td> <br>  <br> - Robot model <br> - CSV files with position, velocity and torque data <br>- Documentation <br> - Hardware (mechanics & electronics) <br> <br> <br> - Place to store your outputs  <br> - Software <br> - C++ code for realtime control <br> <br> - Python code for everything else <br>  <br>  <br>  <br>   <br> <br> &emsp;  &emsp;  &emsp;  &emsp;  &emsp;  &emsp;
           </td>
    </tr>
</table>


## Safety Notes #

DO NOT run the given code to the motors while they are attached to the leg. This code is meant to show a given code flow and test the motors, board, and program setup. Clamp the motors down and run the code on them externally. 

Depending on how the leg is assembled, you may need to adjust the lengths used in the code. More instructions re inside the general definitions section of the program.

## Initial Setup
--------------------------------------------------------------------
CCS is the IDE for TI products and is an easy to sude system. A introduction to CCS can be found at the following link: https://www.youtube.com/watch?v=11lsNYW7zkw&ab_channel=CodeComposer

Hoppy was implemented using version 8 of CCS which can be downloaded at the following link: http://software-dl.ti.com/ccs/esd/documents/ccs_downloads.html. The instructions for installing the software can be found at the second link. 

Once CCS in installed, unzip the Blinky_rtos_FLASHCCS8.zip folder and open the project in CCS. This project should blink an LED on the board after it is flashed onto the board. cpu01_main.c is the main file that you will be working in; this is also the file that you should be in when you build and debug the program.

Once the programming enviroment is working, the file cpu01_main_EXAMPLE.c can be coppied into cpu01_main.c. This program is a starting point for the user to program. It includes an intuitive code structure, working PD control on the hip and knee motors, and contact sensing (analog and binary). The instructions to use the code setup and the different sensing options is in the file with line numbers for each part in the code. 

The wiring diagram that shows how hook up the system is also included in the folder.

----------------------------------------------------------------------------------------------------------------------------

## Usage
Navigate to the folder "Simulator_MATLAB" and set the working directory of MATLAB to be this directory. To start the simulation, run the file
```
...\Simulator_MATLAB\MAIN.m
```
If the code works properly, the following GUI will show up and a video file will be generated.

![](https://i.imgur.com/Ck73nsp.gif)

Please refer to the Code_Instruction.pdf file to navigate through each component of this simulator

## Simulator
---------

Requirements: pybullet

----------------------------------------------------------------------------------------------------------------------------
## CAD
---

The CAD models of the entire setup are inlcuded in SolidWorks and .STEP formats. Video  instructions for mechanical assembly can found at the following link: https://youtu.be/CDxhdjob2C8

The Bill of Materials (BOM), a complete list of components and quantities, is also listed in the folder. 


## References

* [Underactuated Course](docs/reference.md)
* [Hoppy Papper](docs/getting_started.md)
* [Moteus](docs/reference.md)

If you encounter any problems regarding the simulator, please raise an issue under this repo.

Enjoy!
-----------------------------------------------------------------------------------------------------------------------------
