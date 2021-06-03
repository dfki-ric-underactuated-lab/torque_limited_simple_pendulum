<div align="center">

#  Simple Pendulum
</div>

## Structure #

<table>
	<tr>
        <td><ul>
                <li>data/
                <ul>
                    <li>models</li>
                    <li>robot</li>
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
                        <li>filters/</li>
                        <li>model/</li>
                        <li>simulation/</li>
                        <li>trajectory_optimization/</li>
                        <li>utilities/</li>
                </ul>
                </ul>
            </ul></td>    
		<td> <br>  <br>- Machine Learning models <br> - URDF file <br> - CSV files with position, velocity and torque data <br>- Documentation <br> - Hardware (mechanics & electronics) <br> <br> <br> - Place to store your outputs  <br> - Software <br> - C++ code for realtime control <br> <br> - Python code for everything else <br>  <br>  <br>  <br>  <br>  <br> <br> <br> &emsp;  &emsp;  &emsp;  &emsp;  &emsp;  &emsp;
           </td>
    </tr>
</table>

## Introduction #

The project is an open-source, low-cost, robust, and modular kit to get started with dynamic robots. The kit targets lowering the barrier for studying dynamic robots and legged locomotion in real systems. Controlling dynamic motions in real robots present unique challenges to the software and hardware which are often overlooked in conventional robotics courses. This project describes the topics which can be studied using the kit, lists its components, discusses best practices for implementation, presents results from experiments with the simulator and the real system.

LICENSE: All files contained in this repository, unless otherwise noted, are
available under an (...) License:


See the simple pendulum in action here: [torque limited swing up](/hw/simple_pendulum_swingup.mp4)



## Documentation 

* [Getting Started](docs/getting_started.md)
* [Reference](docs/reference.md)

## Safety Notes #

* Brushless motors can be very powerful, moving with tremendous force and speed. Always limit the range of motion, power, force and speed using configurable parameters, current limited supplies, and mechanical design.

* Stay away from the plane in which pendulum is swinging. 

* Make sure you have access to emergency stop while doing experiments. Be extra careful while operating in pure torque control loop. 

Enjoy!
-----------------------------------------------------------------------------------------------------------------------------
