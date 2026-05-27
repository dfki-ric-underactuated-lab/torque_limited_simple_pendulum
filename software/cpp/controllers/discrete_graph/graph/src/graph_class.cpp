#include <iostream>
#include <vector>
#include <map>
#include <limits>
#include <list>
#include <queue>

#include "graph_class.hpp"
#include "utils.hpp"


Graph::Graph(){
    num_vertices = 0;
    num_edges = 0;
}

int Graph::get_num_vertices() {
    return num_vertices;
}

int Graph::get_num_edges() {
    return num_edges;
}

bool Graph::contains_vertex(int val){
    bool contains = false;
    for (int v=0; v<vertices.size(); v++){
        if(vertices[v].id == val){
            contains = true;
            break;
        }
    }
    return contains;
}

int Graph::get_internal_index(int id){
    return internal_index_map[id];
}

int Graph::get_external_index(int ind){
    return internal_index_map_inverse[ind];
}

void Graph::add_vertex(int id) {

    vertices.push_back(Vertex{id});
    //const auto inserted = vertices.push_back(Vertex{id});
    //if (!inserted.second)
    //    throw std::logic_error("existing vertex");
    internal_index_map[id] = num_vertices;
    internal_index_map_inverse[num_vertices] = id;
    //std::cout << "index: " << id << " internal index map: " << internal_index_map[id] << std::endl;
    num_vertices += 1;
    

}
void Graph::add_edge(int src , int dest, double weight, double tau, double dt) {

    //insert vertices with their associated weight
    if (not contains_vertex(src)){
        add_vertex(src);
    }
    if (not contains_vertex(dest)){
        add_vertex(dest);
    }

    Edge e = Edge{src, dest, weight, tau, dt};
    edges.push_back(e);

    int iind = get_internal_index(src);
    std::vector<Edge> temp_edge_list;
    if (iind >= adjacency_edge_list.size()){
        //temp_edge_list.clear();
        temp_edge_list.push_back(e);
        adjacency_edge_list.push_back(temp_edge_list);
    }
    else{
        adjacency_edge_list[iind].push_back(e);
    }

    //const auto inserted = edges.push_back(Edge{src, dest, weight, tau, dt});
    //if (!inserted.second)
    //    throw std::logic_error("existing edge");
    num_edges += 1;
}

//void Graph::relax(std::size_t src, std::size_t dest, int weight)
//{
//    auto& next_dist = vertices[dest].distance;
//    const auto curr_dist = vertices[src].distance + weight;
//    if (curr_dist < next_dist)
//    {
//        next_dist = curr_dist;
//        //update distance in unprocessed queue
//        unprocessed.push( std::make_pair(next_dist, dest));
//    }
//}
//
//void Graph::dijkstra(std::size_t src)
//{
//    //initialize distance of source
//    vertices[src].distance = 0;
//
//    unprocessed.push( std::make_pair(vertices[src].distance, src) );
//    while (!unprocessed.empty())
//    {
//         int curr_vertex_dist = unprocessed.top().first;
//         std::size_t curr_vertex = unprocessed.top().second;
//         unprocessed.pop();
//
//        if (vertices[curr_vertex].color == WHITE)
//        {
//            processed.push_back(curr_vertex);
//        }
//        vertices[curr_vertex].color = BLACK;
//        for (auto& ver: adj_list[curr_vertex])
//        {
//            for(auto edge: edge_weight)
//            {
//                 relax(edge.first.from, edge.first.to, edge.second);
//            }
//        }
//    }
//}
//
//std::ostream& Graph::print_distance(std::ostream& os) const
//{
//    os << "Vertex\t\tDistance from Source\n";
//    for (auto vertex: vertices)
//    {
//        os << vertex.id << "\t\t" << vertex.distance << "\n";
//    }
//    return os;
//}
//
//std::ostream& Graph::print_path(std::ostream& os) const
//{
//    std::cout << "Path : ";
//    for (int i = 0; i < processed.size() - 1; i++)
//    {
//        os << processed[i] << "-->";
//    }
//    os << processed.back() << "\n";
//}
