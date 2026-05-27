#include <filesystem>
#include <iostream>
#include <math.h>
#include <stdlib.h>
// #include <yaml-cpp>
#include <yaml-cpp/node/node.h>
#include <yaml-cpp/node/parse.h>
#include <yaml-cpp/yaml.h>

#include "../../graph/src/utils.hpp"
#include "pendulum_plant.hpp"
#include "pendulum_utils.hpp"

void ComputeTransition(PendulumPlant plant, double pos, double vel, double dt,
                       double ddt, double tau, std::string integrator,
                       double pos_res, double vel_res, double *pos2,
                       double *vel2) {
  double temp_pos2, temp_vel2;
  plant.simulate(0.0, pos, vel, dt, ddt, tau, integrator);
  temp_pos2 = plant.get_position();
  temp_vel2 = plant.get_velocity();
  temp_pos2 = discretize_pos(temp_pos2, pos_res);
  temp_vel2 = discretize_vel(temp_vel2, vel_res);
  *pos2 = temp_pos2;
  *vel2 = temp_vel2;
}

int main(int argc, char *argv[], char *envp[]) {

  // pendulum parameters
  double mass = 0.5;
  double length = 0.5;
  double damping = 0.1;
  double coulomb_friction = 0.0;
  double gravity = 9.81;
  double inertia = mass * length * length;
  double torque_limit = 2.0;

  // exploration parameters
  double dt = 0.1;
  double pos_res = 1.0 * 2 * M_PI / 360.0;
  double vel_res = pos_res / dt;
  double vel_max = 50.0 * vel_res;
  std::string foldername = "Data/results";
  std::string configpath = "Data/results/config.yml";
  std::string integrator = "runge_kutta";
  std::vector<double> torques;
  torques.push_back(-1.0);
  torques.push_back(-0.5);
  torques.push_back(0.0);
  torques.push_back(0.5);
  torques.push_back(1.0);
  int exploration_limit = 1e7;

  YAML::Node config;

  if (argc >= 2) {
    configpath = std::string(argv[1]);

    std::cout << "Reading file at " << configpath << std::endl;
    config = YAML::LoadFile(configpath);

    if (config["mass"]) {
      mass = config["mass"].as<double>();
    }
    if (config["length"]) {
      length = config["length"].as<double>();
    }
    inertia = mass * length * length;
    if (config["damping"]) {
      damping = config["damping"].as<double>();
    }
    if (config["coulomb_friction"]) {
      coulomb_friction = config["coulomb_friction"].as<double>();
    }
    if (config["gravity"]) {
      gravity = config["gravity"].as<double>();
    }
    if (config["inertia"]) {
      inertia = config["inertia"].as<double>();
    }
    if (config["torque_limit"]) {
      torque_limit = config["torque_limit"].as<double>();
    }

    if (config["deltaT"]) {
      dt = config["deltaT"].as<double>();
    }
    if (config["position_resolution"]) {
      pos_res = config["position_resolution"].as<double>() * 2 * M_PI / 360.0;
    }
    vel_res = pos_res / dt;
    if (config["vmax"]) {
      vel_max = config["vmax"].as<double>() * vel_res;
    }
    if (config["tau"]) {
      torques.clear();
      for (int t = 0; t < config["tau"].size(); t++) {
        torques.push_back(config["tau"][t].as<double>());
      }
    }
    if (config["foldername"]) {
      foldername = config["foldername"].as<std::string>();
    }
    if (config["integrator"]) {
      integrator = config["integrator"].as<std::string>();
    }
  } else {
    config = YAML::LoadFile(configpath);
  }

  std::filesystem::create_directory(foldername);

  config["mass"] = mass;
  config["length"] = length;
  config["damping"] = damping;
  config["coulomb_friction"] = coulomb_friction;
  config["gravity"] = gravity;
  config["inertia"] = inertia;
  config["torque_limit"] = torque_limit;

  config["deltaT"] = dt;
  config["position_resolution"] =
      round(1000 * pos_res / (2 * M_PI / 360.0)) / 1000;
  config["velocity_resolution"] =
      round(1000 * vel_res / (2 * M_PI / 360.0)) / 1000;
  config["vmax"] = round(vel_max / vel_res);
  config["tau"] = torques;
  config["foldername"] = foldername;
  config["integrator"] = integrator;
  std::ofstream fout(foldername + "/config.yml");
  fout << config;
  fout.close();

  std::cout << "timestep: " << dt << std::endl;
  std::cout << "position resolution: " << pos_res << std::endl;
  std::cout << "velocity resolution: " << vel_res << std::endl;
  std::cout << "maximum velocity: " << vel_max << std::endl;

  std::cout << "torques: ";
  for (int t = 0; t < torques.size(); t++) {
    std::cout << torques[t] << " ";
  }
  std::cout << std::endl;
  std::cout << "Save data in " << foldername << std::endl;

  int expected_nodes = 2 * M_PI / pos_res * 2 * vel_max / vel_res;
  std::cout << "Expected states: " << expected_nodes << std::endl;
  std::cout << "Expected transitions: " << expected_nodes * torques.size()
            << std::endl;

  PendulumPlant plant;
  plant.set_parameters(mass, length, damping, coulomb_friction, gravity,
                       inertia, torque_limit);

  std::vector<State> state_list;
  State start_state;
  start_state.pos = 0.0;
  start_state.vel = 0.0;
  state_list.push_back(start_state);

  std::vector<int> state_index_list;
  state_index_list.push_back(pos_vel_to_index(start_state.pos, start_state.vel,
                                              pos_res, -M_PI, M_PI, vel_res,
                                              -vel_max, vel_max));

  std::cout << "start state index: " << state_index_list[0] << std::endl;

  std::vector<Transition> transition_list;
  std::vector<std::vector<double>> transition_index_list;

  int ind1, ind2;
  double new_pos, new_vel;
  State state1, state2;
  Transition trans;
  std::vector<double> trans_ind;
  int counter = 0;
  while (counter < state_index_list.size()) {
    for (int t = 0; t < torques.size(); t++) {
      ComputeTransition(plant, state_list[counter].pos, state_list[counter].vel,
                        dt, 0.001, torques[t], integrator, pos_res, vel_res,
                        &new_pos, &new_vel);
      state2.pos = new_pos;
      state2.vel = new_vel;
      if (state2.vel <= vel_max && state2.vel >= -vel_max) {
        ind2 = pos_vel_to_index(state2.pos, state2.vel, pos_res, -M_PI, M_PI,
                                vel_res, -vel_max, vel_max);
        if (std::find(state_index_list.rbegin(), state_index_list.rend(),
                      ind2) == state_index_list.rend()) {
          state_list.push_back(state2);
          state_index_list.push_back(ind2);
        }
        state1.pos = state_list[counter].pos;
        state1.vel = state_list[counter].vel;
        trans.origin = state1;
        trans.head = state2;
        trans.torque = torques[t];
        trans.dt = dt;
        transition_list.push_back(trans);

        ind1 = pos_vel_to_index(state1.pos, state1.vel, pos_res, -M_PI, M_PI,
                                vel_res, -vel_max, vel_max);
        trans_ind.clear();
        trans_ind.push_back(double(ind1));
        trans_ind.push_back(double(ind2));
        trans_ind.push_back(torques[t]);
        trans_ind.push_back(dt);
        transition_index_list.push_back(trans_ind);
      }
    }
    counter += 1;
    if (counter % 1000 == 0) {
      std::cout << counter << std::endl;
    }
    if (counter % exploration_limit == 0) {
      break;
    }
  }
  std::cout << "Found states: " << state_index_list.size() << std::endl;
  std::cout << "Found transitions: " << transition_index_list.size()
            << std::endl;
  CSVReader states_index_reader(foldername + "/states_index.csv");
  states_index_reader.saveData(state_index_list);
  CSVReader transitions_index_reader(foldername + "/transitions_index.csv");
  transitions_index_reader.saveData(transition_index_list);

  return 0;
}
