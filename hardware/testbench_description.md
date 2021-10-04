<div align="center">

#  Hardware & Testbench Description
</div>

The `/hardware` directory contains all information about the hardware that is used to built the simple pednulum test bench, including a bill of materials, step files of the CAD model along with wiring diagrams for the complete set up as well as the CAN bus. Further specifications are noted below or can be found within the documentation under the respective topics.

<div align="right">
<img width="800" src="../hardware/simple_pendulum_CAD.png">  
</div>  


### Physical Parameters of the Pendulum
-------------------------------------------------------------------- 

* Point mass: $`m_p`$= 0.546 Kg 
* Mass of rod, mounting parts and screws: $`m_r`$ = 0.13 Kg 
* Overall mass: $`m`$ = 0.676 Kg
* Length to point mass: $`l`$ = 0.5 m
* Length to COM: $`l_{COM}`$ = 0.045 m 


```math 
l_{COM} = \frac{m_pl + 0.5 m_r l}{m_p + m_r} \, ,
```

### Physical Parameters of the Actuator
--------------------------------------------------------------------  
The AK80-6 actuator from T-Motor is a quasi direct drive with a gear ratio of 6:1 and a peak torque of 12 Nm at the output shaft. The motor controller is basically the same as the one used for MIT Mini-Cheetah, which is described in the documentation from Ben Katz.
- [Ben Katz: MIT Mini-Cheetah Documentation](https://docs.google.com/document/d/1dzNVzblz6mqB3eZVEMyi2MtSngALHdgpTaDJIW_BpS4/edit)

<div align="center">
<img width="600" src="../hardware/ak80-6_img.jpg">  
</div>  

-------------------------------------------------------------------- 

* Voltage = 24 $`V`$
* Current = rated 12 $`A`$, peak 24 $`A`$
* Torque = rated 6 $`Nm`$, peak 12 $`Nm`$ (after the transmission)
* Transmission N = 6 : 1
* Weight = 485 $`g`$
* Dimensions = ⌀ 98 $`mm`$ x 38,5 $`mm`$
* Max. torque to weight ratio = 24 $`Nm/kg`$ (after the transmission) 
* Max. velocity = 38.2 $`rad/s`$ = 365 $`rpm`$ (after the transmission)

### Motor Constants  
-------------------------------------------------------------------- 
 (before the transmission)  

- Motor constant km = 0.2206 $`Nm/ \sqrt{W}`$
- Electric constant ke= 0.009524 $`V/rpm`$ 
- Torque constant kt = 0.091 $`Nm/A`$
- Torque = rated 1,092 $`Nm`$, peak 2,184 $`Nm`$
- Velocity / back-EMF constant kv = 100 $`rpm/V`$
- Max. velocity at 24 $`V`$ = 251.2 $`rad/s`$ = 2400 $`rpm`$
- Motor wiring in $`\nabla`$-delta configuration
- Number of pole pairs = 21
- Resistance phase to phase = 170 $`\pm`$ 5 $`m \Omega`$
- Inductance phase to phase = 57 $`\pm`$ 10 $`\mu H`$
- Rotor inertia Ir = 0.000060719 $`kg \, m^2`$

</br> 

# Electrical Setup

The wiring diagram below shows how the simple pendulum testbench is set up. A main PC is connected to a motor controller board (CubeMars_AK_V1.1) mounted on the actuator (AK80-6 from T-Motor). The communication takes place on a CAN bus with a maximum signal frequency of 1Mbit/sec with the 'classical' CAN protocol. Furthermore, a USB to CAN interface is needed, if the main pc doesn't have a PCI CAN card. Two different devices are used in our setup: the R-LINK module from T-Motor and the PCAN-USB adapter from PEAK systems. The former has CAN and UART connectors at the output, but only works with Windows. The latter only features CAN connection, but also works with Linux. The UART connector of the R-LINK module is usefull to configure and calibrate the AK80-6.   

The actuator requires an input voltage of 24 Volts and consumes up to 24 Amps under full load. A power supply that is able to deliver both and which is used in our test setup is the EA-PS 9032-40 from Elektro-Automatik. A capacitor filters the backEMF coming from the actuator and therefore protects the power supply from high voltage peaks. This wouldn't be necessary if the actuator is powered from a battery pack, as in this case backEMF simply recharges the batteries. The capacitor we use is made of 10x single 2.7V-400F capacitor cells connected in series. A emergency stop button serves as additional safety measure. It disconnects the actuator from the power supply and from the capacitor (Note: If only the power supply is switched off the actuator will continue to run from the energy stored in the capacitor).   

<div align="center">
<img width="800" src="../hardware/sp_wiring_diagram.png">  
</div>  

**Fig. 1:** actuator = AK80-6, controller board = CubeMars_AK_V1.1, power supply = EA-PS 9032-40, capacitor = 10x 2.7V-400F cells connected in series, USB-CAN interfaces = R-LINK module and PCAN-USB adapter.  


# Communication: CAN Bus wiring
Along the CAN bus proper grounding and isolation is required. It is important to not connect ground pins on the CAN bus connectors between different actuators, since this would cause a critical ground loop. The ground pin should only be used to connect to systems with a ground isolated from the power ground. Additionally, isolation between the main pc and the actuators improves the signal quality. When daisy-chaining multiple actuators, only the CAN-High and CAN-Low pins between the drives must be connected. At the end of the chain a 120 Ohm resistor between CAN-H and CAN-L is used to absorb the signals. It prevents the signals from being reflected at the wire ends. The CAN protocol is differential, hence no additional ground reference is needed. The diagram below displays the wiring of the CAN bus.  
  
<br/>
<div align="center">
<img width="600" src="../hardware/can_bus.png">  
</div>    
<br/> 

**Fig. 2:** main pc = CPU, CAN transceiver = CAN XCVR, actuator = AC
