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

#include <typeinfo>

#include "utils.hpp"

struct VertexProperty{
    long ind;
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
                              //boost::no_property,
                              EdgeProperty > DirectedGraph;

typedef boost::graph_traits<DirectedGraph>::vertex_descriptor vertex_descriptor;

template <typename T> std::string type_name();

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
    for (long i=0; i< states.size(); ++i){
        v.ind = long(stod((states[i][0])));
        boost::add_vertex(v, graph);
    }
    std::cout << "Number of vertices = " << num_vertices(graph) << "\n";
    std::cout << "Number of edges = " << num_edges(graph) << "\n";

    //int vertex_counter = 0;
    //std::vector<int> vertex_list;
    //int max = 0;
    //int min = INT_MAX;

    EdgeProperty e;
    for(long i=0; i<transitions.size(); ++i){
        e.weight = 1;
        e.tau = stod(transitions[i][2]);
        e.dt = stod(transitions[i][3]);
        boost::add_edge(long(stod(transitions[i][1])),
                        long(stod(transitions[i][0])),
                        e,
                        graph);
        //if (int(stod(transitions[i][0])) < min){
        //    min = int(stod(transitions[i][0]));
        //}
        //if (int(stod(transitions[i][1])) < min){
        //    min = int(stod(transitions[i][1]));
        //}
        //if (int(stod(transitions[i][0])) > max){
        //    max = int(stod(transitions[i][0]));
        //}
        //if (int(stod(transitions[i][1])) > max){
        //    max = int(stod(transitions[i][1]));
        //}
        //if(find(vertex_list.begin(), vertex_list.end(), int(stod(transitions[i][0]))) == vertex_list.end() ){
        //    vertex_list.push_back(int(stod(transitions[i][0])));
        //}
        //if(find(vertex_list.begin(), vertex_list.end(), int(stod(transitions[i][1]))) == vertex_list.end() ){
        //    vertex_list.push_back(int(stod(transitions[i][1])));
        //}
    }

    //std::cout << "n vertices: " << vertex_list.size() << std::endl;
    //std::cout << "min vertex: " << min << std::endl;
    //std::cout << "max vertex: " << max << std::endl;


    //std::pair<edge_iterator, edge_iterator> ei = edges(graph);
    std::cout << "Number of vertices = " << num_vertices(graph) << "\n";
    std::cout << "Number of edges = " << num_edges(graph) << "\n";
    std::cout << std::endl;

    // Create vectors to store the predecessors (predecessor_map) and the distances from the root (distance_map)
    //auto inds = boost::get(&VertexProperty::ind, graph);
    auto weights = boost::get(&EdgeProperty::weight, graph);
    auto tau_map = boost::get(&EdgeProperty::tau, graph);
    auto dt_map = boost::get(&EdgeProperty::dt, graph);
    std::vector<vertex_descriptor> predecessor_map(num_vertices(graph));
    //std::vector<int> distance_map(num_vertices(graph));
    std::vector<double> trans;
    std::vector<std::vector<double> > transition_map;
    std::vector<boost::graph_traits<DirectedGraph>::vertex_descriptor >::iterator it;
    CSVReader dfile("dummy");

    size_t start_ind = 0;

    std::cout << "Starting Dijkstra Computation... \n";
    //for(int start_ind=0; start_ind<num_vertices(graph); start_ind++){
    //for(auto vi : boost::make_iterator_range(boost::vertices(graph))){
    //for(int vi=18180; vi < 18181; vi++){
    for(long vi=4518000; vi < 4518001; vi++){
    //for(int vi=720000; vi < 720001; vi++){
        //std::cout << start_ind << " " << vi << "\n"; //<< inds(vi) << std::endl;
        //decltype(boost::vertex(vi, graph))::foo = 1;
        //auto vvi = boost::vertex(vi, graph);
        //std::cout << vvi << std::endl;
        //std::cout << start_ind << "of type: " << typeid(start_ind).name() << " " << vi << " of type: " << typeid(vi).name() << " \n"; //<< inds(vi) << std::endl;
        // Evaluate Dijkstra on graph with source start, predecessor_map and distance_map
        transition_map.clear();
        //boost::dijkstra_shortest_paths(graph, start_ind, boost::predecessor_map(&predecessor_map[0]).distance_map(&distance_map[0]));
        boost::dijkstra_shortest_paths(graph, vi, boost::no_named_parameters() .predecessor_map(&predecessor_map[0]). weight_map(weights));
        //boost::dijkstra_shortest_paths(graph, vi, boost::predecessor_map(&predecessor_map[0]). weight_map(weights));
        //boost::dijkstra_shortest_paths(graph, start_ind, boost::predecessor_map(&predecessor_map[0]).distance_map(&distance_map[0]).weight_map(&weight_map[0]));
        for(it=predecessor_map.begin(); it != predecessor_map.end(); ++it){
            trans.clear();
            //std::cout << "(it, pmap[it]): (" << *it << ", " << predecessor_map[*it] << ")" << std::endl;;
            auto e = boost::edge(predecessor_map[*it], *it, graph);
            trans.push_back(double(predecessor_map[*it]));
            if (e.second){
                //trans.push_back(double(predecessor_map[j][1]));
                trans.push_back(tau_map(e.first));
                trans.push_back(dt_map(e.first));
            }
            else{
                trans.push_back(0.0);
                trans.push_back(0.0);
            }
            transition_map.push_back(trans);
            //else{
            //    std::cout << "Edge not found.\n";
            //}

            //for (int i=0; i<transitions.size(); i++){
            //  if (int(stod(transitions[i][0])) == predecessor_map[*it] && int(stod(transitions[i][1])) == *it){
            //      trans.push_back(stod(transitions[i][2]));
            //      trans.push_back(stod(transitions[i][3]));
            //      break;
            //  }
            //}
        }

        dfile = CSVReader(foldername + "/Dijkstra_rev/dijkstra_"+std::to_string(vi)+".csv");
        dfile.writeHeader("go_to_ind, torque, dt");
        //dfile.saveData(predecessor_map);
        dfile.saveData(transition_map);
        if(start_ind % 1000 == 0){
           std::cout << start_ind << std::endl;
        }
        start_ind++;

    }

    return 0;
}
