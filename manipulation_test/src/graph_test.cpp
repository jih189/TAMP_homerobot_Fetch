#include "manipulation_test/experience_graph.hpp"

int main(int,char*[])
{

  uint numOfJoints = 7;

  ExperienceGraph experiencegraph(7);

  std::vector<double> joint1{0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
  unsigned long int vertex_id1 = 1;
  uint constraint_id1 = 1;
  uint node_id1 = 1;

  ExperienceGraph::Vertex v1 = experiencegraph.addVertex(joint1, constraint_id1, node_id1);

  std::vector<double> joint2{0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2};
  unsigned long int vertex_id2 = 2;
  uint constraint_id2 = 2;
  uint node_id2 = 2;

  ExperienceGraph::Vertex v2 = experiencegraph.addVertex(joint2, constraint_id2, node_id2);

  std::vector<double> joint3{0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3};
  unsigned long int vertex_id3 = 3;
  uint constraint_id3 = 3;
  uint node_id3 = 3;

  ExperienceGraph::Vertex v3 = experiencegraph.addVertex(joint3, constraint_id3, node_id3);

  std::vector<double> joint4{0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4};
  unsigned long int vertex_id4 = 4;
  uint constraint_id4 = 4;
  uint node_id4 = 4;

  ExperienceGraph::Vertex v4 = experiencegraph.addVertex(joint4, constraint_id4, node_id4);

  experiencegraph.addEdge(v1, v2, 0.5);

  experiencegraph.addEdge(v2, v3, 0.5);

  experiencegraph.addEdge(v2, v4, 0.5);

  // experiencegraph.printGraph();

  std::vector<ExperienceGraph::Vertex> solution = experiencegraph.findSolution(v4, v1);

  for(auto& v : solution)
  {
    std::cout << experiencegraph.vertex_index_map[v] << std::endl;
  }

  
  return 0;
}

