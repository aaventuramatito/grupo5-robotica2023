#include "graph.h"
#include <ranges>

Graph::Graph()
{
    nodes.push_back(0);
}

int Graph::add_node(int roomID)
{
    nodes.push_back(roomID);
    return roomID;
}

int Graph::add_edge(int n1, int n2) {
    if (std::ranges::find(nodes, n1) != nodes.end() and
        std::ranges::find(nodes, n2) != nodes.end() and
        std::ranges::find(edges, std::make_pair(n1, n2)) == edges.end())
    {
        edges.emplace_back(n1, n2);
        return 1;
    }
    else return -1;
}


void Graph::print()
{
    for (const auto &n : nodes)
    {
        std::cout<< n << " " ;
    }

    std::cout<<std::endl;


    for (const auto &e : edges)
    {
        std::cout<< e.first << " "  << e.second;
    }

    std::cout<<std::endl;

}