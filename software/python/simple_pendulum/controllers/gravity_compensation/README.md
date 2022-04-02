#  Gravity Compensation Control #

Type: Closed loop control

State/action space constraints: -

Optimal: -

Versatility: only compensates for gravitational force acting on the pendulum, no swing-up or stabilization at the upright position

## Theory #

A controller compensating the gravitational force acting on the pendulum. The control function is given by:

<br>

</div>
<div align="center">
<img src="https://render.githubusercontent.com/render/math?math=u(\theta) = mgl \sin(\theta)">
</div>

<br>

where <img src="https://render.githubusercontent.com/render/math?math=u"> is commanded torque, <img src="https://render.githubusercontent.com/render/math?math=m">  is a weight of _0,5 kg_ attached to the rod together with the mass of the rod and the mounting parts, <img src="https://render.githubusercontent.com/render/math?math=l"> is the length of _0,5 m_ of the rod, <img src="https://render.githubusercontent.com/render/math?math=g"> is  gravitational acceleration on earth of <img src="https://render.githubusercontent.com/render/math?math=9.81 \, ms^{-2}">  and <img src="https://render.githubusercontent.com/render/math?math=\theta"> is the current position of the pendulum. 


While the controller is running it actively compensates for the gravitational force acting on the pendulum, therefore the pendulum can be moved as if it was in zero-g.
