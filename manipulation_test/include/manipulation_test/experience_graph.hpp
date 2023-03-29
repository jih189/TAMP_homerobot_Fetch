#include <iostream>                  // for std::cout
#include <utility>                   // for std::pair
#include <algorithm>                 // for std::for_each
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/graphviz.hpp>

#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/datastructures/NearestNeighborsSqrtApprox.h>
#include <ompl/geometric/planners/prm/ConnectionStrategy.h>

class ExperienceGraph
{
    public:
    struct vertex_state_t
    {
      using kind = boost::vertex_property_tag;
    };
    struct vertex_constraint_id_t
    {
      using kind = boost::vertex_property_tag;
    };

    using VertexPropertyType = boost::property<boost::vertex_index_t, long unsigned int,
                               boost::property<vertex_state_t, std::vector<double>,
                               boost::property<vertex_constraint_id_t, std::vector<int>>>>;

    using EdgePropertyType = boost::property<boost::edge_weight_t, double>;

    using GraphType = boost::adjacency_list<boost::vecS, boost::listS, boost::undirectedS, VertexPropertyType, EdgePropertyType>;
    using Vertex = boost::graph_traits<GraphType>::vertex_descriptor;
    using Edge = boost::graph_traits<GraphType>::edge_descriptor;

    // ompl
    using RoadmapNeighbors = std::shared_ptr<ompl::NearestNeighbors<Vertex> >;
    using ConnectionStrategy = std::function<const std::vector<Vertex> &(const Vertex)>;

    RoadmapNeighbors nn_;
    ConnectionStrategy connectionStrategy_;


    boost::property_map<GraphType, boost::vertex_index_t>::type vertex_index_map;
    boost::property_map<GraphType, vertex_state_t>::type vertex_state_map;
    boost::property_map<GraphType, vertex_constraint_id_t>::type vertex_constraint_id_map;
    boost::property_map<GraphType, boost::edge_weight_t>::type edge_weight_map;

    GraphType g_;
    
    int numOfJoints_;
    int numOfCurrentNode;

    // constructor
    ExperienceGraph(int numOfJoints, double maxDistance);
    Vertex addVertex(const std::vector<double>& jointstate, const int& constraint_id);
    Vertex getVertex(const std::vector<double>& jointstate);
    bool hasEdge(const ExperienceGraph::Vertex& v1, const ExperienceGraph::Vertex& v2);
    Edge addEdge(const Vertex& v1, const Vertex& v2, const double& weight);
    Edge addEdge(const Vertex& v1, const Vertex& v2);
    std::vector<Vertex> findSolution(const Vertex& start, const Vertex& goal);
    std::vector<Vertex> getNeighbors(const Vertex& v);
    void printGraph();
};
