#ifndef GRAPH_CLASS_HPP
#define GRAPH_CLASS_HPP

#include <iostream>
#include <vector>
#include <map>
#include <limits>
#include <list>
#include <queue>

struct Vertex
{
    int id;
    Vertex(int id) : id(id) {}
};

struct Edge
{
    int from;
    int to;
    double weight;
    double tau;
    double dt;
    bool operator<(const Edge& other) const
    {
        return std::tie(from, to) < std::tie(other.from, other.to);
    }
};


class Adjacency_edge_list
{
    public:
        std::vector<Vertex> ver;
        std::vector<std::vector<Edge> > edges;

};

class Graph
{

    int num_vertices;
    int num_edges;
    int internal_index_map[1000000];
    int internal_index_map_inverse[1000000];

  public:
    Graph();
    int get_num_vertices();
    int get_num_edges();
    bool contains_vertex(int val);
    void add_vertex(int id);
    void add_edge(int src, int dest, double weight, double tau, double dt);
    int get_internal_index(int ind);
    int get_external_index(int ind);

    std::vector<Vertex> vertices = {};
    std::vector<Edge> edges = {};
    std::vector<std::vector<Edge>> adjacency_edge_list;
};

#endif // GRAPH_CLASS_HPP
