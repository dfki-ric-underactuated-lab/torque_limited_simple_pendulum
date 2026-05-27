#ifndef PENDULUM_UTILS_HPP
#define PENDULUM_UTILS_HPP

#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <math.h>
#include <vector>
#include <iterator>
#include <string>
#include <algorithm>

struct State{
    double pos;
    double vel;
};

struct Transition{
    State origin;
    State head;
    double torque;
    double dt;
};

struct Transition_ind{
    long origin;
    long head;
    double torque;
    double dt;
};


double discretize_pos(double, double);
double discretize_vel(double, double);
long pos_vel_to_index(double, double, double, double, double, double, double, double);
double index_to_pos(long, double, double, double, double, double, double);
double index_to_vel(long, double, double, double, double, double, double);

#endif // PENDULUM_UTILS_HPP
