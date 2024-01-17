#include "graph.h"
#include <ranges>
#include <cppitertools/sliding_window.hpp>
#include <cppitertools/combinations.hpp>
#include <cppitertools/enumerate.hpp>
#include <qdebug.h>


Graph::Graph()
{
    nodes.push_back(0);
}

int Graph::add_node()
{
    nodes.push_back(nodes.size());
    return nodes.size()-1;
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
    std::cout<<"\n";

    std::cout<< "Arcos:  "  <<"\n";
    for (const auto &e : edges)
    {

        std::cout<< e.first << " "  << e.second <<"\n";
    }
    std::cout<<"\n";
}

int Graph::node_count() const {
    return nodes.size();
}


