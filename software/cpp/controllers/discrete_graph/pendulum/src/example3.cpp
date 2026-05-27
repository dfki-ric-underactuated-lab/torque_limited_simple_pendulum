#include <iostream>
#include <vector>

#include <boost/config.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/graph/graph_utility.hpp>

// Create a struct to hold properties for each vertex
struct VertexProperties {
    int ind;

};

// Create a struct to hold properties for each edge
struct EdgeProperties {
    int weight;
    double tau;
    double dt;
};

// Define the type of the graph
typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, VertexProperties, EdgeProperties> Graph;

int main() {
    // Create a graph object
    Graph g;

    // Add vertices
    auto v0 = boost::add_vertex({1}, g),
         v1 = boost::add_vertex({2}, g),
         v2 = boost::add_vertex({3}, g);

    // Add edges
    boost::add_edge(v0, v1, EdgeProperties{22, 0.5, 0.1}, g);
    boost::add_edge(v1, v2, EdgeProperties{33, 0.5, 0.1}, g);

    //boost::print_graph(g, boost::get(&VertexProperties::ind, g));

    // set up a weight map:
    auto weights = boost::get(&EdgeProperties::weight, g);
    //std::cout << "weight: " << weights << std::endl;

    auto e = boost::edge(1, 2, g).first;
    std::cout << "edge: " << e << std::endl;
    std::cout << "weight: " << weights[e] << std::endl;

    // you can pass itprint_graph`enter code here` to dijkstra using direct or named params. Let's do the simplest
    boost::dijkstra_shortest_paths(g, v0, boost::no_named_parameters() .weight_map(weights));
}
