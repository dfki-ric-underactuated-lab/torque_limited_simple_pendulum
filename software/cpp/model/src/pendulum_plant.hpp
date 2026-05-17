#ifndef PENDULUM_PLANT_HPP
#define PENDULUM_PLANT_HPP

#include <string>

struct State{
    double pos;
    double vel;
};

struct Action{
    double u1;
};

class PendulumPlant{
    double mass;
    double length;
    double damping;
    double coulomb_friction;
    double gravity;
    double inertia;
    double torque_limit;

public:

    void set_parameters(double, double, double, double, double, double, double);
    double get_torque_limit();

    double forward_dynamics(double, double, double);
    State rhs(double, double, double, double);

    double calculate_potential_energy(State);
    double calculate_kinetic_energy(State);
    double calculate_total_energy(State);

};

#endif // PENDULUM_PLANT_HPP
