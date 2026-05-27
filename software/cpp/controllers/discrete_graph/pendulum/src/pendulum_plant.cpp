#include <stdlib.h>
#include <iostream>
#include <math.h>

#include "../../graph/src/utils.hpp"
#include "pendulum_utils.hpp"
#include "pendulum_plant.hpp"

void PendulumPlant::set_parameters(double m, double l, double d, double cf, double g, double I, double tl){
    mass = m;
    length = l;
    damping = d;
    coulomb_friction = cf;
    gravity = g;
    inertia = I;
    torque_limit = tl;
}

void PendulumPlant::set_state(double t, double pos, double vel){
    time = t;
    position = pos;
    velocity = vel;
}

double PendulumPlant::forward_dynamics(double pos, double vel, double tau){
    double fric;
    if (vel > 0.0){
        fric = coulomb_friction;
    }
    else if (vel < 0.0) {
        fric = -1.0*coulomb_friction;
    }
    else {
        fric = 0.0;
    }
    double accn = (tau - mass*gravity*length*sin(pos) - damping*vel - fric) / inertia;
    //std::cout << "plant fd: pos: " << pos << "vel: " << vel << "tau" << tau << "acc: " << accn << std::endl;
    return accn;
}
State PendulumPlant::rhs(double t, double pos, double vel, double tau){
    double accn = forward_dynamics(pos, vel, tau);
    State r;
    r.pos = vel;
    r.vel = accn;
    //std::cout << "plant rhs: (" << r.pos << ", " << r.vel << ")" << std::endl;
    return r;
}

State PendulumPlant::euler_integrator(double t, double pos, double vel, double tau, double dt){
     return rhs(t, pos, vel, tau);
}

State PendulumPlant::runge_integrator(double t, double pos, double vel, double tau, double dt){
    State k1, k2, k3, k4, r;
    k1 = rhs(t, pos, vel, tau);
    k2 = rhs(t+0.5*dt, pos+0.5*dt*k1.pos, vel+0.5*dt*k1.vel, tau);
    k3 = rhs(t+0.5*dt, pos+0.5*dt*k2.pos, vel+0.5*dt*k2.vel, tau);
    k4 = rhs(t+dt, pos+dt*k3.pos, vel+dt*k3.vel, tau);
    r.pos = (k1.pos + 2*(k2.pos + k3.pos) + k4.pos) / 6.0;
    r.vel = (k1.vel + 2*(k2.vel + k3.vel) + k4.vel) / 6.0;
    return r;
}

void PendulumPlant::step(double tau, double dt, std::string integrator){
    //std::cout << "Before: tau: " << tau << " torque limit: " << torque_limit << std::endl;
    if (tau > torque_limit){
        tau = torque_limit;
    }
    else if (tau < -torque_limit){
        tau = -torque_limit;
    }
    //std::cout << "After: tau: " << tau << " torque limit: " << torque_limit << std::endl;
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

void PendulumPlant::simulate(double t0, double p0, double v0, double tf, double dt, double tau, std::string integrator){
    set_state(t0, p0, v0);
    while(time < tf - dt*0.5){
        step(tau, dt, integrator);
    }
}
