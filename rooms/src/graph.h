#ifndef ROOMS_GRAPH_H
#define ROOMS_GRAPH_H
#include <vector>
#include <iostream>

class Graph
{
    using Nodes = std::vector<int>;
    using Edges = std::vector<std::pair<int,int>>;
    Nodes nodes;
    Edges edges;

public:
    Graph();
    int add_node();
    int add_edge (int n1, int n2);
    void print();


    bool add_node_and_edge(int currentRoom, int nextRoom);

    size_t get_node_count() const {
        return nodes.size();
    }


    // Método para obtener una copia del vector de nodos
    std::vector<int> getNodes() const {
        return nodes;
    }


    int node_count() const;
};






#endif //ROOMS_GRAPH_H
