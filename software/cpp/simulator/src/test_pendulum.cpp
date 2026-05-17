#include <iostream>
#include <fstream>
#include <sstream>
#include <yaml-cpp/node/node.h>
#include <yaml-cpp/node/parse.h>
#include <yaml-cpp/yaml.h>

#include "../../utils/src/csv_reader.hpp"
#include "../../pendulum_plant/src/pendulum_plant.hpp"
#include "simulator.hpp"

int main(int argc, char *argv[], char *envp[]){

    std::string foldername = ".";
    if (argc >= 1){foldername = std::string(argv[1]);}

    // default parameters

    //pendulum parameters
    double mass = 0.57288;
    double length = 0.5;
    double damping = 0.1;
    double coulomb_friction = 0.0;
    double gravity = 9.81;
    double inertia = mass*length*length;
    double torque_limit = 10.0;

    // simulation parameters
    double dt = 0.01;
    std::string integrator = "runge_kutta";

    // TODO fix yaml loader
    // read parameters from yaml file
    YAML::Node config = YAML::LoadFile(foldername + "/config.yml");
    if (config["mass"]) {mass=config["mass"].as<double>();}
    if (config["length"]) {length=config["length"].as<double>();}
    if (config["damping"]) {damping=config["damping"].as<double>();}
    if (config["coulomb_friction"]) {coulomb_friction=config["coulomb_friction"].as<double>();}
    if (config["gravity"]) {gravity=config["gravity"].as<double>();}
    if (config["inertia"]) {inertia=config["inertia"].as<double>();}
    if (config["torque_limit"]) {torque_limit=config["torque_limit"].as<double>();}
    if (config["deltaT"]) {dt=config["deltaT"].as<double>();}
    if (config["integrator"]){integrator = config["integrator"].as<std::string>();}

    std::cout << "mass " << mass << std::endl;
    std::cout << "length " << length << std::endl;
    std::cout << "damping " << damping << std::endl;
    std::cout << "coublomb_friction " << coulomb_friction << std::endl;
    std::cout << "gravity " << gravity << std::endl;
    std::cout << "inertia " << inertia << std::endl;
    std::cout << "torque_limit " << torque_limit << std::endl;

    PendulumPlant plant;
    plant.set_parameters(mass, length, damping, coulomb_friction, gravity, inertia, torque_limit);
    Simulator sim;
    sim.set_plant(plant);

    CSVReader reader("/home/felix/Work/DFKI/Development/underactuated_lab/pendulum/cpp_pendulum/test/trajectory.csv", ",");
    std::vector<std::vector<double> > trajectory = reader.getDataDouble(1);


    double pos_diff;
    sim.set_state(trajectory[0][0], trajectory[0][1], trajectory[0][2]);
    for (int i=0; i < trajectory.size()-1; i++){
        sim.step(trajectory[i][3], dt, integrator);
        pos_diff = sim.get_position() - trajectory[i+1][1];
        if (pos_diff > 0.001){
            std::cout << trajectory[i][0] << ", " << sim.get_position() << ", " << trajectory[i+1][1] << ", " << pos_diff << std::endl;
        }
    }

}
