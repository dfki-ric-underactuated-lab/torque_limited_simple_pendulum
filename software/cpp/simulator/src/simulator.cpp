#include <stdlib.h>
#include <iostream>
#include <math.h>

#include "simulator.hpp"

void Simulator::set_plant(PendulumPlant pl){
    plant = pl;
}

void Simulator::set_state(double t, double pos, double vel){
    time = t;
    position = pos;
    velocity = vel;
}

State Simulator::euler_integrator(double t, double pos, double vel, double tau, double dt){
     return plant.rhs(t, pos, vel, tau);
}

State Simulator::runge_integrator(double t, double pos, double vel, double tau, double dt){
    State k1, k2, k3, k4, r;
    k1 = plant.rhs(t, pos, vel, tau);
    k2 = plant.rhs(t+0.5*dt, pos+0.5*dt*k1.pos, vel+0.5*dt*k1.vel, tau);
    k3 = plant.rhs(t+0.5*dt, pos+0.5*dt*k2.pos, vel+0.5*dt*k2.vel, tau);
    k4 = plant.rhs(t+dt, pos+dt*k3.pos, vel+dt*k3.vel, tau);
    r.pos = (k1.pos + 2*(k2.pos + k3.pos) + k4.pos) / 6.0;
    r.vel = (k1.vel + 2*(k2.vel + k3.vel) + k4.vel) / 6.0;
    return r;
}

void Simulator::step(double tau, double dt, std::string integrator){
    double tl = plant.get_torque_limit();
    if (tau > tl){
        tau = tl;
    }
    else if (tau < -tl){
        tau = -tl;
    }
    State r;
    if (integrator == "euler"){
        r = euler_integrator(time, position, velocity, tau, dt);
    }
    else if (integrator == "runge_kutta"){
        r = runge_integrator(time, position, velocity, tau, dt);
    }
    position += dt*r.pos;
    velocity += dt*r.vel;
    time += dt;
}

void Simulator::simulate(double t0, double p0, double v0, double tf, double dt, double tau, std::string integrator){
    set_state(t0, p0, v0);
    while(time < tf){
        step(tau, dt, integrator);
    }
}

State Simulator::get_state(){
    State x;
    x.pos = position;
    x.vel = velocity;
    return x;
}
