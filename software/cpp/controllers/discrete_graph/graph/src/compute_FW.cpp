#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <iterator>
#include <string>
#include <algorithm>
#include <math.h>

#include <yaml-cpp/node/node.h>
#include <yaml-cpp/node/parse.h>
#include <yaml-cpp/yaml.h>

#include <boost/config.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/floyd_warshall_shortest.hpp>
#include <boost/graph/exterior_property.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/undirected_graph.hpp>

#include "utils.hpp"

typedef int t_weight;
typedef boost::property<boost::edge_weight_t, t_weight> EdgeWeightProperty;
typedef boost::adjacency_list<boost::listS, boost::vecS, boost::directedS, boost::no_property, EdgeWeightProperty > DirectedGraph;
typedef boost::graph_traits<DirectedGraph>::edge_iterator edge_iterator;
typedef boost::graph_traits<DirectedGraph>::vertex_descriptor vertex_descriptor;
typedef std::pair<int, int> Edge;

typedef boost::exterior_vertex_property<DirectedGraph, t_weight> DistanceProperty;
typedef boost::property_map<DirectedGraph, boost::edge_weight_t>::type WeightMap;
typedef DistanceProperty::matrix_type DistanceMatrix;
typedef DistanceProperty::matrix_map_type DistanceMatrixMap;


//typedef boost::adjacency_list <boost::listS, boost::vecS, boost::directedS, boost::no_property, boost::property<boost::edge_weight_t, int> > graph_t;
//typedef boost::graph_traits<graph_t>::vertex_descriptor vertex_descriptor;
//typedef std::pair<int, int> Edge;


int main(int argc, char *argv[], char *envp[]){

    DirectedGraph graph;

    double dt = 0.1;
    double pos_res = 1.0 * 2*M_PI / 360.0;
    double vel_res = pos_res / dt;
    double vel_max = 50*vel_res;
    std::string foldername = "Data/results";
    std::string integrator = "runge_kutta";

    if (argc >= 1){foldername = std::string(argv[1]);}

    std::cout << "Reading file at " << foldername + "/config.yml" << std::endl;
    YAML::Node config = YAML::LoadFile(foldername + "/config.yml");
    if (config["deltaT"]) {dt=config["deltaT"].as<double>();}
    if (config["position_resolution"]) {pos_res = config["position_resolution"].as<double>()*2*M_PI / 360.0;} 
    vel_res = pos_res / dt;
    if (config["vmax"]) {vel_max = config["vmax"].as<double>()*vel_res;}
    if (config["integrator"]){integrator = config["integrator"].as<std::string>();}

    CSVReader states_reader(foldername + "/states_index.csv");
    CSVReader transition_reader(foldername  + "/transitions_index.csv");
    std::vector<std::vector<std::string> > states= states_reader.getData();
    std::vector<std::vector<std::string> > transitions = transition_reader.getData();

    std::cout << states.size() << std::endl;
    std::cout << transitions.size() << std::endl;

    for(int i=0; i<transitions.size(); ++i){
        boost::add_edge(int(stod(transitions[i][0])), int(stod(transitions[i][1])), 1, graph);
    }

    //std::pair<edge_iterator, edge_iterator> ei = edges(graph);
    std::cout << "Number of vertices = " << num_vertices(graph) << "\n";
    std::cout << "Number of edges = " << num_edges(graph) << "\n";
    std::cout << std::endl;

    WeightMap weight_pmap = boost::get(boost::edge_weight, graph);
    DistanceMatrix distances(num_vertices(graph));
    DistanceMatrixMap dm(distances, graph);

    std::cout << "FW start" << std::endl;
    bool valid = floyd_warshall_all_pairs_shortest_paths(graph, dm, boost::weight_map(weight_pmap));
    std::cout << "FW finished" << std::endl;

 
    return 0;
}
