#include "manipulation_test/experience_graph.hpp"
#include <boost/foreach.hpp>

#define foreach BOOST_FOREACH


ExperienceGraph::ExperienceGraph(int numOfJoints, double maxDistance)
{
    numOfJoints_ = numOfJoints;
    numOfCurrentNode = 0;

    vertex_index_map = boost::get(boost::vertex_index, g_);
    vertex_state_map = boost::get(vertex_state_t(), g_);
    vertex_constraint_id_map = boost::get(vertex_constraint_id_t(), g_);
    edge_weight_map = boost::get(boost::edge_weight, g_);

    nn_ = std::make_shared<ompl::NearestNeighborsSqrtApprox<Vertex>>();
    nn_->setDistanceFunction([this](const Vertex &a, const Vertex &b) {
        std::vector<double> joint1 = vertex_state_map[a];
        std::vector<double> joint2 = vertex_state_map[b];
        double diff = 0;
        for(int i = 0; i < numOfJoints_; i++)
            diff += pow(abs(joint1[i] - joint2[i]), 2);
        return diff;
    });
    connectionStrategy_ = ompl::geometric::KBoundedStrategy<Vertex>(5, maxDistance, nn_);
}

// add a vertex to the graph
// if the vertex already exists, return the vertex.
ExperienceGraph::Vertex ExperienceGraph::addVertex(const std::vector<double>& jointstate, const int& constraint_id)
{
    // check the joint state size
    if(jointstate.size() > numOfJoints_)
    {
        std::cout << "The joint state size is larger than the number of joints" << std::endl;
        return NULL;
    }

    // check if the vertex already exists
    Vertex result = getVertex(jointstate);
    if(result)
    {
        // if exists, then add the constraint id to the vertex
        if(std::count(vertex_constraint_id_map[result].begin(), vertex_constraint_id_map[result].end(), constraint_id) == 0)
        {
            // add the constraint id to the vertex if it does not exist
            vertex_constraint_id_map[result].push_back(constraint_id);
        }
        return result;
    }
    else
    {
        // if not exists, then add the vertex to the graph
        Vertex m = boost::add_vertex(g_);
        vertex_state_map[m] = jointstate;
        vertex_index_map[m] = numOfCurrentNode;
        vertex_constraint_id_map[m].push_back(constraint_id);

        nn_->add(m);

        numOfCurrentNode++;
        return m;
    }
}

// get the vertex from the graph
// if the vertex does not exist, return NULL
ExperienceGraph::Vertex ExperienceGraph::getVertex(const std::vector<double>& jointstate)
{
    if(jointstate.size() > numOfJoints_)
    {
        std::cout << "The joint state size is larger than the number of joints" << std::endl;
        return NULL;
    }

    foreach (Vertex n, boost::vertices(g_))
    {
        std::vector<double> currentjoint = vertex_state_map[n];

        for(int i = 0; i < numOfJoints_; i++)
        {
            if(currentjoint[i] != jointstate[i])
                break;
            else if(i == numOfJoints_ - 1)
                return n;
        }
    }
    return NULL;
}

bool ExperienceGraph::hasEdge(const ExperienceGraph::Vertex& v1, const ExperienceGraph::Vertex& v2)
{
    return boost::edge(v1, v2, g_).second;
}

ExperienceGraph::Edge ExperienceGraph::addEdge(const ExperienceGraph::Vertex& v1, const ExperienceGraph::Vertex& v2, const double& weight)
{
    Edge e = boost::add_edge(v1, v2, g_).first;
    edge_weight_map[e] = weight;
    return e;
}

ExperienceGraph::Edge ExperienceGraph::addEdge(const ExperienceGraph::Vertex& v1, const ExperienceGraph::Vertex& v2)
{
    Edge e = boost::add_edge(v1, v2, g_).first;
    std::vector<double> joint1 = vertex_state_map[v1];
    std::vector<double> joint2 = vertex_state_map[v2];
    // calculate the difference between two vectors
    double diff = 0;
    for(int i = 0; i < numOfJoints_; i++)
        diff += pow(abs(joint1[i] - joint2[i]), 2);
    edge_weight_map[e] = std::sqrt(diff);
    return e;
}

std::vector<ExperienceGraph::Vertex> ExperienceGraph::findSolution(const ExperienceGraph::Vertex& start, const ExperienceGraph::Vertex& goal)
{ // Dijkstra's algorithm
    // need to verfiy later
    std::vector<Vertex> predecessors(boost::num_vertices(g_));
    std::vector<double> distances(boost::num_vertices(g_));

    boost::dijkstra_shortest_paths(g_, start,
                                    boost::predecessor_map(boost::make_iterator_property_map(predecessors.begin(), boost::get(boost::vertex_index, g_))).
                                    distance_map(boost::make_iterator_property_map(distances.begin(), boost::get(boost::vertex_index, g_))));

    std::vector<Vertex> path;

    if(distances[vertex_index_map[goal]] == std::numeric_limits<double>::max())
    {
        return path;
    }

    Vertex current = goal;

    while(current != start)
    {
    path.push_back(current);
    current = predecessors[vertex_index_map[current]];
    }

    path.push_back(start);

    std::reverse(path.begin(), path.end());
    return path;
}

std::vector<ExperienceGraph::Vertex> ExperienceGraph::getNeighbors(const ExperienceGraph::Vertex& v)
{
    return connectionStrategy_(v);
}

void ExperienceGraph::printGraph()
{
    write_graphviz(std::cout, g_);
}