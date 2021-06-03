<div align="center">

#  Simple Pendulum
</div>

<br/>
### Test setup
-------------------------------------------------------------------- 
The wiring diagram below shows how the simple pendulum testbench is set up. A main PC is connected to a motor controller board (CubeMars_AK_V1.1) mounted on the actuator (AK80-6 from T-Motor). The communication takes place on a CAN bus with a maximum signal frequency of 1Mbit/sec with the 'classical' CAN protocol. Furthermore, a USB to CAN interface is needed, if the main pc doesn't have a PCI CAN card. Two different devices are used in our setup: the R-LINK module from T-Motor and the PCAN-USB adapter from PEAK systems. The former has CAN and UART connectors at the output, but only works with Windows. The latter only features CAN connection, but works with Linux. The UART connector of the R-LINK module is usefull to configure and calibrate the AK80-6.   

The actuator requires an input voltage of 24 Volts and consumes up to 24 Amps under full load. A power supply that is able to deliver both and which is used in our test setup is the EA-PS 9032-40 from Elektro-Automatik. A capacitor filters the backEMF coming from the actuator and therefore protects the power supply from high voltage peaks. This wouldn't be necessary if the actuator is powered from a battery pack, as in this case backEMF simply recharges the batteries. The capacitor we use is made of 10x single 2.7V-400F capacitor cells connected in series. A emergency stop button serves as additional safety measure. It disconnects the actuator from the power supply and from the capacitor (Note: If only the power supply is switched off the actuator will continue to run from the energy stored in the capacitor).   

<div align="center">
<img width="600" src="wiring_diagram.png">  
</div>  

**Fig. 1:** actuator = AK80-6, controller board = CubeMars_AK_V1.1, power supply = EA-PS 9032-40, capacitor = 10x 2.7V-400F cells connected in series, USB-CAN interfaces = R-LINK module and PCAN-USB adapter.  

<br/>
### CAN Bus
-------------------------------------------------------------------- 

<div align="center">
<img width="400" src="can_bus.png">  
</div>    
<br/> 
<br/>
##  Configuration: R-Link Config Tool
-------------------------------------------------------------------- 

- Silabs: [CP210x Universal Windows Driver](https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers)  
- CH341: [Sparkfun - How to install CH340 drivers](https://learn.sparkfun.com/tutorials/how-to-install-ch340-drivers/all)  
- User manual & configuration tool: [https://store-en.tmotor.com/goods.php?id=1085](https://store-en.tmotor.com/goods.php?id=1085)

Before starting to use the R-Link device make sure you have downloaded the `CP210x Universal Windows Driver` from silabs. If this isn't working properly follow the instructions at sparkfun on how to install ch340 drivers. You have to download the `CH 341SER (EXE)` file from the sparkfun webpage. Notice that you first have to select **uninstall** in the CH341 driver menu to uninstall old drivers before you are able to install the new driver. The configuration tool software for the R-LINK module can be downloaded on the T-Motors website.  


1. Wire the R-LINK module as shown in figure 1. A USB to micro USB cable connects a pc with the R-LINK module and the 5pin cable goes between the R-LINK module and the Motor.
 
2. Connect the AK80-6 motor to a power supply (24V, 12A) and do not cut off the power before the setting is completed.  <br/> 

3. Start the R-Link Config Tool application (only runs on Windows).  

4. Select serial port: **USB-Serial_CH340,wch,cp** along with an appropriate baud rate (both 921600 and 115200 Bd should work). If the serial port option USB-Serial_CH340,wch,cp does not show up, your pc can’t establish a connection to the R-LINK module due to remaining driver issues.  

3. Choose the desired motor settings on the left side of the config tool GUI. Enter the correct CAN ID of the motor under `MotorSelectEnter`. A label on the motor shows the ID.

	* Velocity: **5 rad/s** is a relatively slow speed of revolution, hence it offers a good starting point.
	* Torque: be careful setting a fixed torque, because the friction inside the motor decreases with the speed of revolution. Therefore a fixed torque commonly leads to either no movement at all or accelerates the motor continuously.  
   <br/> 

4. Start the plotting by ticking the boxes of position, velocity, torque and select  `Display`

5. Press `Run` to start recording the plots.

6. `Enter M_Mode` to control the motor. This is indicated by a color change of the plot line, from red to green.

7. In order to push changes in the settings to the motor, press `Send Once`.  
> **WARNING:** This button does not work reliably. Usually it has to be activated several times 
> before the setting changes actually apply on the motor.   

8. Stop the motor inside the M-Mode by setting the velocity to 0 and pressing `Send Once` until the changes apply.

9. `Exit M_Mode` to exit the control mode of the motor.  
> **WARNING:** The next time you start the motor control with `Enter M_Mode` the motor will restart with the exact same settings as you left the control mode with `Exit M_Mode`. This is especially dangerous if a weight is attached to the pendulum and the motor control was left with high velocity or torque settings.  
  
10. Use `Stop` to deactivate the plotting.

 </div>
 <br/>
  
 ## Debugging
--------------------------------------------------------------------

Error messages that showed up during the configuration procedure, such as `UVLO` (VM undervoltage lockout) and `OTW` (Thermal warning and shutdown), could be interpreted with the help of the datasheet for the DRV8353M 100-V Three-Phase Smart Gate Driver from Texas Instruments:

**Datasheet:** [DRV8353M](https://www.ti.com/lit/ds/symlink/drv8353m.pdf) (on the first Page under: 1. Features) 


 ## Getting started
 --------------------------------------------------------------------

* Clone the python driver for T-motors direct drive: https://git.hb.dfki.de/underactuated-robotics/mini-cheetah-motor/python-motor-driver
* Modify the `.bashrc` file to add the driver to your python path. Make sure you restart your terminal after this step.
```
# mini-cheetah driver
export PYTHONPATH=~/path/from/home/to/underactuated-robotics/python-motor-driver:${PYTHONPATH}
```
* Make sure you setup your can interface first. The easiest way to do this is to run `sh setup_caninterface.sh` from the `mini-cheetah-motor/python-motor-driver` folder. Read more about it here: https://git.hb.dfki.de/underactuated-robotics/mini-cheetah-motor/python-motor-driver/-/blob/master/README.md
* To run an offline computed swingup trajectory, use: `python3 swingup_control.py`. The script assumes can id as `'can0'` and motor id as `0x01`. If these parameters differ, please modify them within the script.

  
 ## PID-Controller
 --------------------------------------------------------------------
* [PID-Controller](docs/getting_started.md)


 ## Further Tutorials
 --------------------------------------------------------------------

- T-MOTOR: [https://www.youtube.com/watch?v=hbqQCgebaF8](https://www.youtube.com/watch?v=hbqQCgebaF8)  
- Skyentific: [https://www.youtube.com/watch?v=HzY9vzgPZkA](https://www.youtube.com/watch?v=HzY9vzgPZkA)  

