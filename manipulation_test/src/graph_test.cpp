#include <iostream>                  // for std::cout
#include <utility>                   // for std::pair
#include <algorithm>                 // for std::for_each
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/graphviz.hpp>

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
    struct vertex_node_id_t
    {
      using kind = boost::vertex_property_tag;
    };
    using Vertex = boost::adjacency_list_traits<boost::vecS, boost::listS, boost::undirectedS>::vertex_descriptor;
    using Graph = boost::adjacency_list<
          boost::vecS, boost::listS, boost::undirectedS,
              boost::property<boost::vertex_index_t, unsigned long int,
              boost::property<vertex_state_t, std::vector<float>,
              boost::property<vertex_constraint_id_t, unsigned int,
              boost::property<vertex_node_id_t, unsigned int>>>>, 
              boost::property<boost::edge_weight_t, double>>;
    using Edge = boost::graph_traits<Graph>::edge_descriptor;

    Graph g_;
    boost::property_map<Graph, boost::vertex_index_t>::type indexProperty_;
    boost::property_map<Graph, vertex_state_t>::type jointStateProperty_;
    boost::property_map<Graph, vertex_constraint_id_t>::type constraintProperty_;
    boost::property_map<Graph, vertex_node_id_t>::type nodeProperty_;
    boost::property_map<Graph, boost::edge_weight_t>::type weightProperty_;
    

    int numOfJoints_;
    int numOfCurrentNode;

    // constructor
    ExperienceGraph(int numOfJoints)
    {
      numOfJoints_ = numOfJoints;
      numOfCurrentNode = 0;
    }


    Vertex addVertex(const std::vector<float>& jointstate, const uint constraint_id, const uint node_id)
    {
      Vertex m = boost::add_vertex(g_);
      jointStateProperty_[m] = jointstate;
      constraintProperty_[m] = constraint_id;
      nodeProperty_[m] = node_id;

      numOfCurrentNode++;

      return m;
    }
};








// void printJointState(const std::vector<float>& j)
// {
//   for(int i = 0; i < j.size(); i++)
//     std::cout << j[i] << " ";
//   std::cout << std::endl;
// }

// void printall(Graph g)
// {
//   typedef boost::property_map<Graph, vertex_state_t>::type JointMap;
//   JointMap jm = boost::get(vertex_state_t(), g);

//   typedef boost::graph_traits<Graph>::vertex_iterator vertex_iter;
//   std::pair<vertex_iter, vertex_iter> vp;
//   for (vp = vertices(g); vp.first != vp.second; ++vp.first)
//     printJointState(jm[*vp.first]);

// }

int main(int,char*[])
{

  uint numOfJoints = 7;

  ExperienceGraph experiencegraph(7);

  std::vector<float> joint1{0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
  unsigned long int vertex_id1 = 1;
  uint constraint_id1 = 1;
  uint node_id1 = 1;

  experiencegraph.addVertex(joint1, constraint_id1, node_id1);


  // Graph g_;

  // boost::property_map<Graph, boost::vertex_index_t>::type indexProperty_;
  // boost::property_map<Graph, vertex_state_t>::type stateProperty_;
  // // boost::property_map<Graph, vertex_constraint_id_t>::type constraintProperty_ =  boost::get(vertex_constraint_id_t(), g_);
  // // boost::property_map<Graph, vertex_node_id_t>::type nodeProperty_ = boost::get(vertex_node_id_t(), g_);

  // // boost::property_map<Graph, boost::edge_weight_t>::type weightProperty_ = boost::get(boost::edge_weight_t(), g_);

  // std::vector<float> joint1{0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
  // unsigned long int vertex_id1 = 1;
  // uint constraint_id1 = 1;
  // uint node_id1 = 1;
  

  // std::vector<float> joint2{0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2};
  // unsigned long int vertex_id2 = 2;
  // uint constraint_id2 = 2;
  // uint node_id2 = 2;

  // double edge_weight_1_2 = 0.6;

  // Vertex m = boost::add_vertex(g_);
  // indexProperty_[m] = node_id1;
  // stateProperty_[m] = joint1;

  // Vertex n = boost::add_vertex(g_);
  // indexProperty_[n] = node_id2;
  // stateProperty_[n] = joint2;

  // boost::add_edge(m, n, edge_weight_1_2, g_);

  // printall( g_);


  // write_graphviz(std::cout, g_);

  // printJoints(joint1);
  
  return 0;
}

