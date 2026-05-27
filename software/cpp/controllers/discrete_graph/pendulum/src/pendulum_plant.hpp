#ifndef PENDULUM_PLANT_HPP
#define PENDULUM_PLANT_HPP

#include "../../graph/src/utils.hpp"
#include "pendulum_utils.hpp"

/*
struct State{
    double pos;
    double vel;
};
*/

class PendulumPlant{
    double mass;
    double length;
    double damping;
    double coulomb_friction;
    double gravity;
    double inertia;
    double torque_limit;

    double position;
    double velocity;
    double time;

    double forward_dynamics(double, double, double);
    //std::tuple<double, double> rhs(double, double, double, double);
    //std::tuple<double, double> euler_integrator(double, double, double, double, double);
    //std::tuple<double, double> runge_integrator(double, double, double, double, double);
    State rhs(double, double, double, double);
    State euler_integrator(double, double, double, double, double);
    State runge_integrator(double, double, double, double, double);
public:

    void set_parameters(double, double, double, double, double, double, double);
    double get_position() {return position;};
    double get_velocity() {return velocity;};

    void set_state(double, double, double);
    void step(double, double, std::string);
    void simulate(double, double, double, double, double, double, std::string);

};

#endif // PENDULUM_PLANT_HPP
