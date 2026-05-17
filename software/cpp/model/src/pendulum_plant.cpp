#include <stdlib.h>
#include <iostream>
#include <math.h>

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

double PendulumPlant::get_torque_limit(){
    return torque_limit;
}

double PendulumPlant::calculate_potential_energy(State x){
    return mass*gravity*length*(1.0 - cos(x.pos));
}

double PendulumPlant::calculate_kinetic_energy(State x){
    return 0.5*mass*pow(length*x.vel, 2);
}

double PendulumPlant::calculate_total_energy(State x){
    return calculate_kinetic_energy(x) + calculate_potential_energy(x);
}
