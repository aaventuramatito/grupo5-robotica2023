#include "graph.h"
#include <ranges>

Graph::Graph()
{
    nodes.push_back(0);
}

int Graph::add_node()
{
    nodes.push_back(nodes.size());
    return nodes.size();
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
        std::cout<< e.first << " "  << e.second <<std::endl;
    }
    std::cout<<std::endl;
}

int Graph::node_count() const {
    return nodes.size();
}


