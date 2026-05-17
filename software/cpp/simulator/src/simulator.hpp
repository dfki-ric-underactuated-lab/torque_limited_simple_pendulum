#ifndef SIMULATOR_HPP
#define SIMULATOR_HPP

#include <string>
#include "../../model/src/pendulum_plant.hpp"

class Simulator{

    PendulumPlant plant;

    double position;
    double velocity;
    double time;

    State euler_integrator(double, double, double, double, double);
    State runge_integrator(double, double, double, double, double);

public:

    void set_plant(PendulumPlant);
    double get_position() {return position;};
    double get_velocity() {return velocity;};
    State get_state();

    void set_state(double, double, double);
    void step(double, double, std::string);
    void simulate(double, double, double, double, double, double, std::string);

};

#endif // SIMULATOR_HPP
