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
    int add_node(int roomID);
    int add_edge (int n1, int n2);
    void print();
};

#endif //ROOMS_GRAPH_H
