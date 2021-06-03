<div align="center">

#  Simple Pendulum
</div>


### Equations of Motions


```math
\tau = m l^2 \ddot{\theta} + mgl sin(\theta) + B \dot{\theta}  
```
where:
* $`m`$: mass of the pendulum
* $`l`$: length of pendulum
* $`g`$: acceleration due to gravity
* $`\theta`$: angle of the pendulum with respect to its stable fixed point configuration
* $`\dot{\theta}`$: angular velocity
* $`\ddot{\theta}`$: angular acceleration
* $`B`$: damping
<br/>

### Total Energy of the Pendulum
-------------------------------------------------------------------- 

* Kinetic Energy (K) = 
```math 
K = \frac{1}{2}ml^2\dot{\theta}^2 
```
* Potential Energy (U) 
```math 
U = - mgl\cos(\theta)
```
* Total Energy (E) 
```math 
E = K + U
```
<br/>

### Physical Parameters of the Pendulum
-------------------------------------------------------------------- 

* Length = 0.5 m
* Mass of cylinder mounting = 0.05 Kg
* Radius of cylinder mounting = 0.02 m 
* Mass of rod = 0.08 Kg
* Mass of 0.5 Kg weight including weight mount = 0.546 Kg
* Radius of 0.5 Kg weight = 0.045 m 

<br/>

### Physical Parameters of the Actuator
--------------------------------------------------------------------  

* Voltage = 24 $`V`$
* Current = rated 12 $`A`$, peak 24 $`A`$
* Torque = rated 6 $`Nm`$, peak 12 $`Nm`$ (after the transmission)
* Transmission N = 6 : 1
* Weight = 485 $`g`$
* Dimensions = ⌀ 98 $`mm`$ x 38,5 $`mm`$
* Max. torque to weight ratio = 24 $`Nm/kg`$ (after the transmission) 
* Max. velocity = 38.2 $`rad/s`$ = 365 $`rpm`$ (after the transmission)

-------------------------------------------------------------------- 
The AK80-6 actuator from T-Motor is a quasi direct drive with a gear ratio of 6:1 and a peak torque of 12 Nm at the output shaft.  

<div align="center">
<img width="600" src="../hw/ak80-6_img.jpg">  
</div>  

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

### Motor Controller (same as MIT Mini-Cheetah)
-------------------------------------------------------------------- 
- [Ben Katz: Documentation](https://docs.google.com/document/d/1dzNVzblz6mqB3eZVEMyi2MtSngALHdgpTaDJIW_BpS4/edit)





