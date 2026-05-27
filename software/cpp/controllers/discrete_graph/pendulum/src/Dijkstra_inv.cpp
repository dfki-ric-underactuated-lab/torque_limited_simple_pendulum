#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>

#include <yaml-cpp/node/node.h>
#include <yaml-cpp/node/parse.h>
#include <yaml-cpp/yaml.h>

#include <boost/algorithm/string.hpp>
#include <boost/config.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/exterior_property.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/property_map/property_map.hpp>

#include "../../graph/src/utils.hpp"
#include "pendulum_utils.hpp"

struct VertexProperty {
  long ind;
};

struct EdgeProperty {
  double weight;
  double tau;
  double dt;
};

typedef boost::adjacency_list<boost::listS, boost::vecS, boost::directedS,
                              VertexProperty, EdgeProperty>
    DirectedGraph;

typedef boost::graph_traits<DirectedGraph>::vertex_descriptor vertex_descriptor;

int main(int argc, char *argv[], char *envp[]) {

  DirectedGraph graph;

  double dt = 0.1;
  double pos_res = 1.0 * 2 * M_PI / 360.0;
  double vel_res = pos_res / dt;
  double vel_max = 50 * vel_res;
  std::string foldername = "Data/results";
  std::string integrator = "runge_kutta";
  // std::vector<double> torques;

  if (argc >= 2) {
    foldername = std::string(argv[1]);
  }

  std::cout << "Reading file at " << foldername + "/config.yml" << std::endl;
  YAML::Node config = YAML::LoadFile(foldername + "/config.yml");
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
  // if (config["tau"]){
  //     torques.clear();
  //     for (int t=0; t < config["tau"].size(); t++){
  //         torques.push_back(config["tau"][t].as<double>());
  //     }
  // }
  // if (config["foldername"]){foldername =
  // config["foldername"].as<std::string>();}
  if (config["integrator"]) {
    integrator = config["integrator"].as<std::string>();
  }

  CSVReader states_reader(foldername + "/states_index.csv");
  CSVReader transition_reader(foldername + "/transitions_index.csv");
  std::vector<std::vector<std::string>> states = states_reader.getData();
  std::vector<std::vector<std::string>> transitions =
      transition_reader.getData();

  std::cout << "Loaded graph with" << states.size() << " States and "
            << transitions.size() << " transitions." << std::endl;

  VertexProperty v1, v2;
  EdgeProperty e;
  for (long i = 0; i < transitions.size(); ++i) {
    v1.ind = (long)stod(transitions[i][1]); // swapped origin and head
    v2.ind = (long)stod(transitions[i][0]);
    e.weight = 1; /// abs(stod(transitions[i][2]));
    // e.weight = std::fabs(stod(transitions[i][2]));
    e.tau = stod(transitions[i][2]);
    e.dt = stod(transitions[i][3]);
    boost::add_edge(v1.ind, v2.ind, e, graph);
  }

  std::cout << "Number of vertices = " << num_vertices(graph) << "\n";
  std::cout << "Number of edges = " << num_edges(graph) << "\n";
  std::cout << std::endl;

  double goal_pos = discretize_pos(M_PI, pos_res);
  double goal_vel = discretize_vel(0.0, vel_res);
  std::cout << "goal pos " << goal_pos << std::endl;
  std::cout << "goal vel " << goal_vel << std::endl;

  long goal_state_index = pos_vel_to_index(goal_pos, goal_vel, pos_res, -M_PI,
                                           M_PI, vel_res, -vel_max, vel_max);
  std::cout << "goal state " << goal_state_index << std::endl;

  // Create descriptor for the source node
  vertex_descriptor goal = vertex(goal_state_index, graph);
  std::cout << " goal: " << goal << std::endl;

  // Create vectors to store the predecessors (predecessor_map) and the
  // distances from the root (distance_map)
  std::vector<vertex_descriptor> predecessor_map(num_vertices(graph));
  std::vector<long> distance_map(num_vertices(graph));
  // std::vector<EdgeWeightProperty> weight_map(num_vertices(graph));
  auto weights = boost::get(&EdgeProperty::weight, graph);
  auto tau_map = boost::get(&EdgeProperty::tau, graph);
  // auto dt_map = boost::get(&EdgeProperty::dt, graph);

  // Evaluate Dijkstra on graph with source start, predecessor_map and
  // distance_map
  std::cout << "Starting Dijkstra search..." << "\n";
  boost::dijkstra_shortest_paths(graph, goal,
                                 boost::no_named_parameters()
                                     .predecessor_map(&predecessor_map[0])
                                     .weight_map(weights));
  std::cout << "Dijkstra search done" << "\n";
  // CSVReader dfile(foldername + "/dijkstra_inv.csv");
  // dfile.saveData(predecessor_map);

  std::vector<double> trans;
  std::vector<std::vector<double>> transition_map;

  for (long v = 0; v < (long)predecessor_map.size(); ++v) {
    trans.clear();
    auto e = boost::edge(predecessor_map[v], v, graph);
    trans.push_back(double(predecessor_map[v]));
    if (e.second) {
      trans.push_back(tau_map(e.first));
    } else {
      trans.push_back(0.0);
    }
    transition_map.push_back(trans);
  }

  std::filesystem::create_directory(foldername + "/Dijkstra_rev");

  CSVReader dfile = CSVReader(foldername + "/Dijkstra_rev/dijkstra_" +
                              std::to_string(num_vertices(graph)) + ".csv");
  // std::to_string(goal) + ".csv");
  // dfile.writeHeader("go_to_ind, torque, dt");
  dfile.writeHeader("go_to_ind, torque");
  dfile.saveData(transition_map);

  return 0;
}
