<div align="center">

#  Torque Limited Simple Pendulum
</div>

## Introduction #

The project is an open-source and low-cost kit to get started with underactuated robotics. The kit targets lowering the entry barrier for studying underactuation in real systems which is often overlooked in conventional robotics courses. It implements a torque limited simple pendulum built using a quasi-direct drive motor which allows for a low friction, torque limited setup. This project describes the offline and online control methods which can be studied using the kit, lists its components, discusses best practices for implementation, presents results from experiments with the simulator and the real system.

<div align="center">
<img width="300" src="../hw/simple_pendulum_CAD.png">
</div>

See the simple pendulum in action here: [torque limited swing up](/hw/simple_pendulum_swingup.mp4)


## Documentation 

* [Testbench Setup](docs/testbench_setup.md)
* [Getting Started](docs/getting_started.md)
* [Usage](docs/usage.md)
* [Reference](docs/reference.md)


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


## Safety Notes #

* Brushless motors can be very powerful, moving with tremendous force and speed. Always limit the range of motion, power, force and speed using configurable parameters, current limited supplies, and mechanical design.

* Stay away from the plane in which pendulum is swinging. 

* Make sure you have access to emergency stop while doing experiments. Be extra careful while operating in pure torque control loop. 

## Contributors #

* [Shivesh Kumar](https://robotik.dfki-bremen.de/en/about-us/staff/shku02.html) (Project Supervisor)
* [Felix Wiebe](https://robotik.dfki-bremen.de/en/about-us/staff/fewi01.html)
* [Daniel Harnack](https://robotik.dfki-bremen.de/en/about-us/staff/daha03.html)
* [Jonathan Babel](https://robotik.dfki-bremen.de/en/about-us/staff/joba02.html)
* [Heiner Peters](https://robotik.dfki-bremen.de/en/about-us/staff/hepe02.html)
* [Shubham Vyas](https://robotik.dfki-bremen.de/en/about-us/staff/shvy01/)
* [Melya Boukheddimi](https://robotik.dfki-bremen.de/en/about-us/staff/mebo01/)
* [Mihaela Popescu](https://robotik.dfki-bremen.de/en/about-us/staff/mipo02/)

Feel free to contact us if you have questions about the test bench. Enjoy!

-----------------------------------------------------------------------------------------------------------------------------
