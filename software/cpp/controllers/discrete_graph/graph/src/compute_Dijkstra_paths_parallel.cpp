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

#include <boost/algorithm/string.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/exterior_property.hpp>
#include <boost/config.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/property_map/property_map.hpp>

//#include <typeinfo>
#include <pthread.h>

#include "utils.hpp"

struct VertexProperty{
    int ind;
};

struct EdgeProperty{
    int weight;
    double tau;
    double dt;
};

typedef boost::adjacency_list<boost::listS,
                              boost::vecS,
                              boost::directedS,
                              VertexProperty,
                              EdgeProperty > DirectedGraph;

typedef boost::graph_traits<DirectedGraph>::vertex_descriptor vertex_descriptor;

template <typename T> std::string type_name();

struct dijkstra_arguments{
    DirectedGraph graph;
    int start_index;
    boost::adj_list_edge_property_map<boost::directed_tag, int, int&, long unsigned int, EdgeProperty, int EdgeProperty::*> &weights;
    boost::adj_list_edge_property_map<boost::directed_tag, double, double&, long unsigned int, EdgeProperty, double EdgeProperty::*> &tau_map;
    boost::adj_list_edge_property_map<boost::directed_tag, double, double&, long unsigned int, EdgeProperty, double EdgeProperty::*> &dt_map;
    std::string foldername;
};

void *compute_dijkstra(void *ptr){

    dijkstra_arguments *args;
    args = (dijkstra_arguments *) ptr;

    std::vector<double> trans;
    std::vector<std::vector<double> > transition_map;
    std::vector<boost::graph_traits<DirectedGraph>::vertex_descriptor >::iterator it;
    std::vector<vertex_descriptor> predecessor_map(num_vertices((*args).graph));
    boost::dijkstra_shortest_paths((*args).graph, (*args).start_index, boost::no_named_parameters() .predecessor_map(&predecessor_map[0]). weight_map((*args).weights));
    for(it=predecessor_map.begin(); it != predecessor_map.end(); ++it){
        trans.clear();
        auto e = boost::edge(predecessor_map[*it], *it, (*args).graph);
        trans.push_back(double(predecessor_map[*it]));
        if (e.second){
            trans.push_back((*args).tau_map(e.first));
            trans.push_back((*args).dt_map(e.first));
        }
        else{
            trans.push_back(0.0);
            trans.push_back(0.0);
        }
        transition_map.push_back(trans);
    }

    CSVReader dfile = CSVReader((*args).foldername + "/Dijkstra/dijkstra_"+std::to_string((*args).start_index)+".csv");
    dfile.writeHeader("go_to_ind, torque, dt");
    dfile.saveData(transition_map);
}

int main(int argc, char *argv[], char *envp[]){

    DirectedGraph graph;

    std::string foldername = "Data/results";
    if (argc >= 1){foldername = std::string(argv[1]);}

    CSVReader states_reader(foldername + "/states_index.csv");
    CSVReader transition_reader(foldername  + "/transitions_index.csv");
    std::vector<std::vector<std::string> > states = states_reader.getData();
    std::vector<std::vector<std::string> > transitions = transition_reader.getData();

    std::cout << "Loaded graph with" << states.size() << " States and " << transitions.size() << " transitions." << std::endl;

    VertexProperty v;
    for (int i=0; i< states.size(); ++i){
        v.ind = int(stod((states[i][0])));
        boost::add_vertex(v, graph);
    }
    std::cout << "Number of vertices = " << num_vertices(graph) << "\n";
    std::cout << "Number of edges = " << num_edges(graph) << "\n";

    int vertex_counter = 0;
    std::vector<int> vertex_list;
    int max = 0;
    int min = INT_MAX;

    EdgeProperty e;
    for(int i=0; i<transitions.size(); ++i){
        e.weight = 1;
        e.tau = stod(transitions[i][2]);
        e.dt = stod(transitions[i][3]);
        boost::add_edge(int(stod(transitions[i][1])),
                        int(stod(transitions[i][0])),
                        e,
                        graph);
    }

    //std::pair<edge_iterator, edge_iterator> ei = edges(graph);
    std::cout << "Number of vertices = " << num_vertices(graph) << "\n";
    std::cout << "Number of edges = " << num_edges(graph) << "\n";
    std::cout << std::endl;

    // Create vectors to store the predecessors (predecessor_map) and the distances from the root (distance_map)
    auto weights = boost::get(&EdgeProperty::weight, graph);
    auto tau_map = boost::get(&EdgeProperty::tau, graph);
    auto dt_map = boost::get(&EdgeProperty::dt, graph);
    //weights = 1;
    //tau_map = 1;
    //dt_map = 1;

    //std::vector<vertex_descriptor> predecessor_map(num_vertices(graph));
    //std::vector<double> trans;
    //std::vector<std::vector<double> > transition_map;
    //std::vector<boost::graph_traits<DirectedGraph>::vertex_descriptor >::iterator it;
    //CSVReader dfile("dummy");

    size_t start_ind = 0;

    pthread_t thread[2];
    int ret[2];

    //dijkstra_arguments *args[2] {{graph, 0, weights, tau_map, dt_map, foldername},
    //                            {graph, 0, weights, tau_map, dt_map, foldername}};
    dijkstra_arguments *args[2];

    std::cout << "Starting Dijkstra Computation... \n";
    //for(int start_ind=0; start_ind<num_vertices(graph); start_ind++){
    //for(auto vi : boost::make_iterator_range(boost::vertices(graph))){
    //for(int vi=18000; vi < 18001; vi++){
    for(int vi=0; vi < 2; vi++){
        std::cout << "set pars" << std::endl;
        (*args[vi]).graph = graph;
        std::cout << "set start ind" << std::endl;
        args[vi]->start_index = vi;
        args[vi]->weights = weights;
        args[vi]->tau_map = tau_map;
        args[vi]->dt_map = dt_map;
        args[vi]->foldername = foldername;
        
        std::cout << "start thread" << std::endl;
        ret[vi] = pthread_create(&thread[vi], NULL, compute_dijkstra, (void*) args[vi]);
        //compute_dijkstra(graph, vi, weights, tau_map, dt_map, foldername); 
        //if(start_ind % 1000 == 0){
        //   std::cout << start_ind << std::endl;
        //}
        //start_ind++;

    }
    for(int vi=0; vi < 2; vi++){
        pthread_join(thread[vi], NULL);
    }

    return 0;
}
