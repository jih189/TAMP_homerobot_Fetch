#include <iostream>                  // for std::cout
#include <utility>                   // for std::pair
#include <algorithm>                 // for std::for_each
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/graphviz.hpp>

#include <moveit/robot_trajectory/robot_trajectory.h>

class TaskPlanner
{
    struct ActionState
    {
        unsigned int foliation_id;
        unsigned int manifold_id;
        bool is_in_manipulation_manifold;
        std::vector<float> joint_values;
        float value;
    };

    struct ActionNode
    {
        unsigned int action_node_id;
        unsigned int configuration_id;
        ActionState in_state;
        ActionState out_state;
        float reward;
        bool isBetweenFoliations;
        moveit_msgs::RobotTrajectory motion_trajectory;
        std::vector<unsigned int> next_action_node_ids;
        std::vector<float> next_action_success_probabilities;
        unsigned int policy;
    };
    
    public:
    TaskPlanner(std::vector<unsigned int> number_of_intermediate_manifolds_, std::vector<unsigned int> number_of_manipulation_manifolds_);

    // if the action is between two foliation, the motion trajectory should be start from last foliation and end at the next foliation.    
    void addActionBetweenFoliations(moveit_msgs::RobotTrajectory motion_trajectory, unsigned int last_foliation_id, unsigned int last_manipulation_id, unsigned int next_foliation_id, unsigned int next_manipulation_id, float reward);

    // if the action is between two manifolds, the motion trajectory should be start from intermediate manifold and end at the next manipulation manifold.
    void addActionBetweenManifolds(moveit_msgs::RobotTrajectory motion_trajectory, unsigned int intermedaite_manifold_id, unsigned int manipulation_manifold_id, unsigned int foliation_id, float reward);
    
    void policyIteration();

    void policyEvaluation();

    void constructMDPGraph();

    private:
    unsigned int current_configuration_id;
    unsigned int current_action_node_id;
    std::vector<unsigned int> number_of_intermediate_manifolds;
    std::vector<unsigned int> number_of_manipulation_manifolds;
    unsigned int number_of_foliations;

    std::vector<std::vector<std::vector<unsigned int>>> manipulation_manifold_out_nodes;
    std::vector<std::vector<std::vector<unsigned int>>> manipulation_manifold_in_nodes;

    std::vector<std::vector<std::vector<unsigned int>>> intermediate_manifold_out_nodes;
    std::vector<std::vector<std::vector<unsigned int>>> intermediate_manifold_in_nodes;

    std::vector<ActionNode> action_nodes;
    float discount_factor;
    float max_error;
};